/*
 * rtl_tcp_soapy, an rtl_tcp compatible I/Q spectrum server that is backed by
 * any SoapySDR supported SDR device instead of a Realtek RTL2832 dongle.
 *
 * The wire protocol (12 byte handshake, unsigned 8 bit IQ stream and the
 * 5 byte command packets) is the same as the one implemented by rtl_tcp, so
 * unmodified rtl_tcp clients (gr-osmosdr, gqrx, SDR#, ...) can connect to it.
 *
 * Copyright (C) 2026 by the rtl-sdr contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <SoapySDR/Constants.h>
#include <SoapySDR/Device.h>
#include <SoapySDR/Errors.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Types.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define DEFAULT_ADDR		"127.0.0.1"
#define DEFAULT_PORT		"1234"
#define DEFAULT_FREQ_HZ		100000000.0
#define DEFAULT_RATE_HZ		2048000.0
#define DEFAULT_TUNER_TYPE	5	/* RTLSDR_TUNER_R820T */
#define DEFAULT_READ_ELEMS	16384
#define READ_TIMEOUT_US		100000

/*
 * Some devices (for example the PlutoSDR with the Tezuka firmware) cannot
 * stream below a certain sample rate because the receive FIR needs a valid
 * setting.  When a client asks for a lower rate, the server runs the hardware
 * at an integer multiple of the requested rate and decimates the samples.  The
 * default threshold is the common 1.024 MHz minimum.
 */
#define DEFAULT_MIN_HW_RATE	1024000.0
#define MAX_DECIM_FACTOR	4096

/*
 * Decimation filter parameters.  A Kaiser window is used because it gives a
 * controlled trade-off between passband ripple, stopband attenuation and
 * transition width.  96 taps per decimation step keeps the response flat to
 * within 0.02 dB over almost the whole output band (the previous Hamming
 * design started rolling off at about half the band) while still suppressing
 * aliases by DECIM_STOPBAND_DB a little above the band edge.
 */
#define DECIM_STOPBAND_DB	60.0	/* stopband attenuation target */
#define DECIM_TAPS_PER_FACTOR	96	/* filter length grows with the factor */

/* Structure sent to the client right after a connection is accepted. */
typedef struct {			/* size must be 12 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t tuner_gain_count;
} __attribute__((packed)) dongle_info_t;

/*
 * Gain values (in tenths of a dB) that librtlsdr reports for an R820T tuner.
 * rtl_tcp clients use the index into this table (command 0x0d) or a value in
 * tenths of a dB (command 0x04); both are mapped onto the real SoapySDR gain.
 */
static const int r82xx_gains[] = {
	0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254,
	280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496
};
#define R82XX_GAIN_COUNT ((int)(sizeof(r82xx_gains) / sizeof(r82xx_gains[0])))

/*
 * Number of gain steps advertised to rtl_tcp clients.  The values are derived
 * from the actual device gain range at runtime (see build_gain_table), so the
 * highest index always selects the highest gain the device supports.
 */
#define SPOOF_GAIN_COUNT 29

/* How the samples delivered by SoapySDR are mapped to unsigned 8 bit IQ. */
typedef enum {
	CONV_CS8_TO_U8,		/* signed 8 bit: flip the sign bit */
	CONV_CU8,		/* already unsigned 8 bit: use as is */
	CONV_CS16_TO_U8,	/* signed 16 bit scaled by full scale */
	CONV_CF32_TO_U8		/* normalized float scaled by full scale */
} conv_kind_t;

/* Device configuration selected on the command line. */
typedef struct {
	double freq;
	double rate;
	double bandwidth;
	int have_bandwidth;
	double gain;
	int have_gain;
	int biastee;
} init_config_t;

/*
 * A streaming FIR decimator.  The hardware runs at factor times the rate the
 * client asked for, and every factor-th filtered sample is sent on.
 */
typedef struct {
	size_t factor;		/* decimation factor, 1 means disabled */
	size_t ntaps;
	float *coeff;
	float *hist_i;		/* circular delay line for I */
	float *hist_q;		/* circular delay line for Q */
	size_t pos;		/* next write position in the delay lines */
	size_t phase;		/* input samples counted modulo factor */
} decimator_t;

typedef struct {
	SoapySDRDevice *dev;
	SoapySDRStream *stream;
	size_t channel;
	conv_kind_t conv;
	double full_scale;
	size_t read_elems;
	size_t in_elem_bytes;
	init_config_t init;
	int *gain_table;	/* spoofed gain values in tenths of a dB */
	size_t gain_count;
	double gain_min_db;
	double gain_max_db;
	decimator_t decim;
	int decim_enabled;	/* set to 0 with --no-decimate */
	double min_hw_rate;
} sdr_ctx_t;

static volatile sig_atomic_t g_do_exit = 0;
static volatile int g_client_fd = -1;

static void sighandler(int signum)
{
	(void)signum;
	g_do_exit = 1;
	if (g_client_fd >= 0)
		shutdown(g_client_fd, SHUT_RDWR);
}

static void usage(const char *prog)
{
	printf("rtl_tcp_soapy, an rtl_tcp compatible I/Q server for SoapySDR devices\n\n");
	printf("Usage:\t%s [options]\n\n", prog);
	printf("Device options:\n");
	printf("\t--args <args>      SoapySDR device arguments, e.g. \"driver=uhd\"\n");
	printf("\t                   (default: first device found)\n");
	printf("\t-c, --channel <n>  RX channel index (default: 0)\n");
	printf("\t-f, --freq <Hz>    initial center frequency (default: %.0f)\n", DEFAULT_FREQ_HZ);
	printf("\t-s, --samplerate <Hz>  initial sample rate (default: %.0f)\n", DEFAULT_RATE_HZ);
	printf("\t-b, --bandwidth <Hz>   RF filter bandwidth (default: driver default)\n");
	printf("\t-g, --gain <dB>    initial gain in dB, enables manual gain mode\n");
	printf("\t                   (default: automatic gain control)\n");
	printf("\t-T, --bias-tee     enable bias tee at startup\n");
	printf("Server options:\n");
	printf("\t--listen <addr>    listen address as host:port, :port or host\n");
	printf("\t-a, --addr <addr>  listen address (default: %s)\n", DEFAULT_ADDR);
	printf("\t-p, --port <port>  listen port (default: %s)\n", DEFAULT_PORT);
	printf("Sample rate options:\n");
	printf("\t--no-decimate      pass low sample rates to the device directly\n");
	printf("\t                   instead of decimating from a higher rate\n");
	printf("\t--min-hw-rate <Hz> minimum hardware sample rate (default: %.0f)\n",
	       DEFAULT_MIN_HW_RATE);
	printf("Compatibility options:\n");
	printf("\t--tuner-type <n>   spoofed RTL-SDR tuner type (default: %d, R820T)\n", DEFAULT_TUNER_TYPE);
	printf("\t-D, --direct-sampling  accepted for compatibility, ignored\n");
	printf("\t-h, --help         show this help\n\n");
	printf("Example:\n");
	printf("\t%s --args \"driver=uhd\" --listen 127.0.0.1:1234\n", prog);
	exit(1);
}

static const char *soapy_last_error(void)
{
	const char *err = SoapySDRDevice_lastError();
	return (err && err[0]) ? err : "unknown error";
}

static void *xmalloc(size_t size)
{
	void *p = malloc(size);
	if (!p) {
		fprintf(stderr, "rtl_tcp_soapy: out of memory\n");
		exit(1);
	}
	return p;
}

/* Parse a frequency/sample rate accepting optional k/M/G suffixes. */
static double parse_hz(const char *s)
{
	char *end = NULL;
	double v = strtod(s, &end);

	if (end == s)
		return 0.0;
	if (*end == 'k' || *end == 'K')
		v *= 1e3;
	else if (*end == 'm' || *end == 'M')
		v *= 1e6;
	else if (*end == 'g' || *end == 'G')
		v *= 1e9;

	return v;
}

/*
 * Split a --listen value into an address and a port.  Accepts "host:port",
 * ":port", "host", "[::1]:port" and "[::1]".  When no port is present the
 * existing value (i.e. the default or a previous -p) is kept.
 */
static void parse_listen(const char *val, char *addr, size_t addr_len,
			 char *port, size_t port_len)
{
	if (val[0] == '[') {
		const char *close = strchr(val, ']');
		if (close) {
			size_t len = (size_t)(close - (val + 1));
			if (len >= addr_len)
				len = addr_len - 1;
			memcpy(addr, val + 1, len);
			addr[len] = '\0';
			if (close[1] == ':' && close[2])
				snprintf(port, port_len, "%s", close + 2);
			return;
		}
	}

	{
		const char *first = strchr(val, ':');
		const char *last = strrchr(val, ':');

		if (first && first == last) {
			size_t len = (size_t)(first - val);
			if (len >= addr_len)
				len = addr_len - 1;
			memcpy(addr, val, len);
			addr[len] = '\0';
			if (first[1])
				snprintf(port, port_len, "%s", first + 1);
		} else {
			/* no colon, or a bare IPv6 literal without a port */
			snprintf(addr, addr_len, "%s", val);
		}
	}
}

static int send_all(int fd, const void *buf, size_t len)
{
	const uint8_t *p = (const uint8_t *)buf;

	while (len > 0) {
		ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			return -1;
		}
		if (n == 0)
			return -1;
		p += (size_t)n;
		len -= (size_t)n;
	}
	return 0;
}

static void set_gain_db(sdr_ctx_t *s, double db)
{
	if (SoapySDRDevice_setGain(s->dev, SOAPY_SDR_RX, s->channel, db) != 0)
		fprintf(stderr, "rtl_tcp_soapy: setGain(%.1f dB) failed: %s\n",
			db, soapy_last_error());
}

/* Free the FIR decimator state and disable decimation. */
static void decimator_free(decimator_t *d)
{
	free(d->coeff);
	free(d->hist_i);
	free(d->hist_q);
	d->coeff = NULL;
	d->hist_i = NULL;
	d->hist_q = NULL;
	d->factor = 1;
	d->ntaps = 0;
	d->pos = 0;
	d->phase = 0;
}

/* Zeroth order modified Bessel function of the first kind (series form). */
static double bessel_i0(double x)
{
	double term = 1.0, sum = 1.0;
	double y = 0.25 * x * x;
	int k;

	for (k = 1; k < 64; k++) {
		term *= y / ((double)k * (double)k);
		sum += term;
		if (term < 1e-16 * sum)
			break;
	}
	return sum;
}

/*
 * Kaiser window shape parameter for a target stopband attenuation.  The
 * piecewise formulas are the standard Kaiser estimates.
 */
static double kaiser_beta(double atten_db)
{
	if (atten_db > 50.0)
		return 0.1102 * (atten_db - 8.7);
	if (atten_db >= 21.0)
		return 0.5842 * pow(atten_db - 21.0, 0.4) +
		       0.07886 * (atten_db - 21.0);
	return 0.0;
}

/*
 * Design a Kaiser windowed-sinc low pass filter and prepare the delay lines.
 * The -6 dB point is placed at the output Nyquist frequency, so the transition
 * band straddles the edge of the requested band: the passband stays flat to
 * within 0.02 dB up to roughly 95% of Nyquist and aliases are suppressed by
 * DECIM_STOPBAND_DB from roughly 105% on.  The filter is normalised so that
 * its DC gain is exactly unity, so decimation does not change the signal
 * level.
 */
static void decimator_design(decimator_t *d, size_t factor)
{
	double atten_db = DECIM_STOPBAND_DB;
	double fc = 0.5 / (double)factor;
	double beta, i0beta, sum = 0.0;
	size_t n, i;

	decimator_free(d);

	d->factor = factor;
	d->ntaps = DECIM_TAPS_PER_FACTOR * factor + 1;
	n = d->ntaps;

	d->coeff = xmalloc(n * sizeof(float));
	d->hist_i = xmalloc(n * sizeof(float));
	d->hist_q = xmalloc(n * sizeof(float));

	beta = kaiser_beta(atten_db);
	i0beta = bessel_i0(beta);

	for (i = 0; i < n; i++) {
		double m = (double)i - (double)(n - 1) / 2.0;
		double sinc = (m == 0.0) ? 1.0 :
			sin(2.0 * M_PI * fc * m) / (2.0 * M_PI * fc * m);
		double t = (n == 1) ? 0.0 :
			(2.0 * (double)i / (double)(n - 1)) - 1.0;
		double window = bessel_i0(beta * sqrt(1.0 - t * t)) / i0beta;

		d->coeff[i] = (float)(2.0 * fc * sinc * window);
		sum += d->coeff[i];
	}

	for (i = 0; i < n; i++)
		d->coeff[i] /= (float)sum;

	memset(d->hist_i, 0, n * sizeof(float));
	memset(d->hist_q, 0, n * sizeof(float));
	d->pos = 0;
	d->phase = 0;
}

/*
 * Filter unsigned 8-bit IQ samples and keep every factor-th output sample.
 * The filter state stays between calls, so block boundaries are seamless.
 * Returns the number of complex samples written to out.
 */
static size_t decimator_process(decimator_t *d, const uint8_t *in, size_t elems,
				uint8_t *out)
{
	size_t n = d->ntaps;
	size_t k, t, produced = 0;

	for (k = 0; k < elems; k++) {
		float sum_i = 0.0f, sum_q = 0.0f;
		size_t idx;
		int vi, vq;

		d->hist_i[d->pos] = (float)((int)in[2 * k] - 128);
		d->hist_q[d->pos] = (float)((int)in[2 * k + 1] - 128);
		d->pos = (d->pos + 1 == n) ? 0 : d->pos + 1;

		if (++d->phase < d->factor)
			continue;
		d->phase = 0;

		idx = d->pos;
		for (t = 0; t < n; t++) {
			idx = (idx == 0) ? n - 1 : idx - 1;
			sum_i += d->coeff[t] * d->hist_i[idx];
			sum_q += d->coeff[t] * d->hist_q[idx];
		}

		vi = (int)lrintf(sum_i);
		vq = (int)lrintf(sum_q);
		if (vi > 127)
			vi = 127;
		else if (vi < -128)
			vi = -128;
		if (vq > 127)
			vq = 127;
		else if (vq < -128)
			vq = -128;

		out[2 * produced] = (uint8_t)(vi + 128);
		out[2 * produced + 1] = (uint8_t)(vq + 128);
		produced++;
	}

	return produced;
}

/*
 * Set the sample rate.  If the requested rate is below the hardware minimum
 * and decimation is enabled, the hardware runs at an integer multiple of the
 * requested rate and the samples are decimated back to the exact rate the
 * client asked for.
 */
static void set_sample_rate(sdr_ctx_t *s, double requested)
{
	double hw = requested;
	size_t factor = 1;

	if (s->decim_enabled && requested > 0.0 &&
	    s->min_hw_rate > 0.0 && requested < s->min_hw_rate) {
		factor = (size_t)ceil(s->min_hw_rate / requested);
		if (factor < 1)
			factor = 1;
		if (factor > MAX_DECIM_FACTOR) {
			fprintf(stderr, "rtl_tcp_soapy: decimation factor limited "
				"to %d\n", MAX_DECIM_FACTOR);
			factor = MAX_DECIM_FACTOR;
		}
		hw = requested * (double)factor;
	}

	if (SoapySDRDevice_setSampleRate(s->dev, SOAPY_SDR_RX, s->channel,
					 hw) != 0) {
		fprintf(stderr, "rtl_tcp_soapy: failed to set sample rate: %s\n",
			soapy_last_error());
		return;
	}

	if (factor > 1) {
		decimator_design(&s->decim, factor);
		printf("sample rate %.0f Hz (hardware %.0f Hz, decimate by %zu)\n",
		       requested,
		       SoapySDRDevice_getSampleRate(s->dev, SOAPY_SDR_RX,
						    s->channel), factor);
	} else {
		decimator_free(&s->decim);
		printf("sample rate set to %.0f Hz\n",
		       SoapySDRDevice_getSampleRate(s->dev, SOAPY_SDR_RX,
						    s->channel));
	}
}

/*
 * (Re)apply the configuration selected on the command line.  This is called
 * at startup and again for every new client, so that a client which does not
 * set the sample rate (several DAB/rtl_tcp clients rely on the rtl_tcp default
 * of 2048000 Hz) always starts from a known state instead of inheriting the
 * settings left behind by the previous client.
 */
static void apply_config(sdr_ctx_t *s)
{
	const init_config_t *cfg = &s->init;
	SoapySDRRange range;

	set_sample_rate(s, cfg->rate);

	if (SoapySDRDevice_setFrequency(s->dev, SOAPY_SDR_RX, s->channel,
					cfg->freq, NULL) != 0)
		fprintf(stderr, "rtl_tcp_soapy: failed to set center frequency: %s\n",
			soapy_last_error());
	else
		printf("tuned to %.0f Hz\n",
		       SoapySDRDevice_getFrequency(s->dev, SOAPY_SDR_RX, s->channel));

	if (cfg->have_bandwidth) {
		if (SoapySDRDevice_setBandwidth(s->dev, SOAPY_SDR_RX, s->channel,
						cfg->bandwidth) != 0)
			fprintf(stderr, "rtl_tcp_soapy: failed to set bandwidth: %s\n",
				soapy_last_error());
	}

	if (SoapySDRDevice_hasGainMode(s->dev, SOAPY_SDR_RX, s->channel)) {
		if (SoapySDRDevice_setGainMode(s->dev, SOAPY_SDR_RX, s->channel,
					       cfg->have_gain ? false : true) != 0)
			fprintf(stderr, "rtl_tcp_soapy: failed to set gain mode: %s\n",
				soapy_last_error());
	}

	if (cfg->have_gain)
		set_gain_db(s, cfg->gain);

	/* A previous client may have changed the ppm correction. */
	SoapySDRDevice_setFrequencyCorrection(s->dev, SOAPY_SDR_RX, s->channel, 0.0);

	range = SoapySDRDevice_getGainRange(s->dev, SOAPY_SDR_RX, s->channel);
	printf("gain range: %.1f .. %.1f dB (step %.3f), current %.1f dB%s\n",
	       range.minimum, range.maximum, range.step,
	       SoapySDRDevice_getGain(s->dev, SOAPY_SDR_RX, s->channel),
	       cfg->have_gain ? "" : " (AGC)");

	if (cfg->biastee) {
		SoapySDRDevice_writeSetting(s->dev, "biastee", "true");
		SoapySDRDevice_writeSetting(s->dev, "bias_tee", "true");
		printf("bias tee enabled\n");
	}
}

/*
 * Build the gain table advertised to rtl_tcp clients from the device's real
 * gain range.  The table spans [minimum, maximum] linearly over SPOOF_GAIN_COUNT
 * steps so that the highest gain index selects the device's highest gain.
 * If the device reports no usable range we fall back to the R820T table.
 */
static void build_gain_table(sdr_ctx_t *s)
{
	SoapySDRRange range = SoapySDRDevice_getGainRange(s->dev, SOAPY_SDR_RX,
							  s->channel);
	size_t i;

	if (range.maximum > range.minimum) {
		s->gain_count = SPOOF_GAIN_COUNT;
		s->gain_table = xmalloc(s->gain_count * sizeof(int));
		s->gain_min_db = range.minimum;
		s->gain_max_db = range.maximum;
		for (i = 0; i < s->gain_count; i++) {
			double db = range.minimum +
				(range.maximum - range.minimum) *
				(double)i / (double)(s->gain_count - 1);
			s->gain_table[i] = (int)lrint(db * 10.0);
		}
	} else {
		s->gain_count = (size_t)R82XX_GAIN_COUNT;
		s->gain_table = xmalloc(sizeof(r82xx_gains));
		memcpy(s->gain_table, r82xx_gains, sizeof(r82xx_gains));
		s->gain_min_db = r82xx_gains[0] / 10.0;
		s->gain_max_db = r82xx_gains[R82XX_GAIN_COUNT - 1] / 10.0;
	}

	printf("advertising %zu gain steps from %.1f to %.1f dB\n",
	       s->gain_count, s->gain_min_db, s->gain_max_db);
}

/* Map a SoapySDR stream format to our converter and its element size. */
static int format_to_conv(const char *fmt, conv_kind_t *conv, size_t *elem_bytes,
			  double *full_scale)
{
	if (!strcmp(fmt, SOAPY_SDR_CS8)) {
		*conv = CONV_CS8_TO_U8;
		*elem_bytes = 2;
		*full_scale = 127.0;
		return 0;
	}
	if (!strcmp(fmt, SOAPY_SDR_CU8)) {
		*conv = CONV_CU8;
		*elem_bytes = 2;
		*full_scale = 255.0;
		return 0;
	}
	if (!strcmp(fmt, SOAPY_SDR_CS16)) {
		*conv = CONV_CS16_TO_U8;
		*elem_bytes = 4;
		*full_scale = 32768.0;
		return 0;
	}
	if (!strcmp(fmt, SOAPY_SDR_CF32)) {
		*conv = CONV_CF32_TO_U8;
		*elem_bytes = 8;
		*full_scale = 1.0;
		return 0;
	}
	return -1;
}

/*
 * Pick a stream format and set up how we convert its samples to the unsigned
 * 8 bit IQ expected by rtl_tcp.  CS8 is preferred (signed 8 bit, so we only
 * have to flip the sign bit), then CU8, then CS16/CF32 converted by us.  The
 * device's native format is always kept as a last resort.
 */
static int setup_stream(sdr_ctx_t *s)
{
	static const char *preferred[] = {
		SOAPY_SDR_CS8, SOAPY_SDR_CU8, SOAPY_SDR_CS16, SOAPY_SDR_CF32
	};
	const char *candidates[8];
	int num_candidates = 0;
	double native_fs = 1.0;
	char *native;
	char **advertised;
	size_t num_advertised = 0;
	size_t i, j;
	int have_stream = 0;

	native = SoapySDRDevice_getNativeStreamFormat(s->dev, SOAPY_SDR_RX,
						      s->channel, &native_fs);
	advertised = SoapySDRDevice_getStreamFormats(s->dev, SOAPY_SDR_RX,
						     s->channel, &num_advertised);

	for (i = 0; i < sizeof(preferred) / sizeof(preferred[0]); i++) {
		int listed = (advertised == NULL);

		for (j = 0; j < num_advertised; j++) {
			if (!strcmp(preferred[i], advertised[j])) {
				listed = 1;
				break;
			}
		}
		if (listed)
			candidates[num_candidates++] = preferred[i];
	}

	/* Keep the native format as a last resort if it was not listed. */
	if (native) {
		int already = 0;

		for (i = 0; i < (size_t)num_candidates; i++) {
			if (!strcmp(candidates[i], native)) {
				already = 1;
				break;
			}
		}
		if (!already)
			candidates[num_candidates++] = native;
	}

	for (i = 0; i < (size_t)num_candidates; i++) {
		conv_kind_t conv;
		size_t elem_bytes;
		double full_scale;

		if (format_to_conv(candidates[i], &conv, &elem_bytes,
				   &full_scale) != 0)
			continue;

		/*
		 * If we requested the native format there is no conversion, so
		 * a native CS16/CF32 keeps the device's own full scale.
		 */
		if (native && !strcmp(candidates[i], native) &&
		    (!strcmp(native, SOAPY_SDR_CS16) ||
		     !strcmp(native, SOAPY_SDR_CF32)))
			full_scale = native_fs;

		s->stream = SoapySDRDevice_setupStream(s->dev, SOAPY_SDR_RX,
						       candidates[i], NULL, 0, NULL);
		if (s->stream) {
			s->conv = conv;
			s->in_elem_bytes = elem_bytes;
			s->full_scale = full_scale;
			printf("using stream format %s (full scale %.0f)\n",
			       candidates[i], full_scale);
			have_stream = 1;
			break;
		}
	}

	if (advertised)
		SoapySDRStrings_clear(&advertised, num_advertised);
	SoapySDR_free(native);

	if (!have_stream) {
		fprintf(stderr, "rtl_tcp_soapy: no usable stream format: %s\n",
			soapy_last_error());
		return -1;
	}

	s->read_elems = SoapySDRDevice_getStreamMTU(s->dev, s->stream);
	if (s->read_elems == 0 || s->read_elems > DEFAULT_READ_ELEMS)
		s->read_elems = DEFAULT_READ_ELEMS;
	printf("stream MTU %zu elements, reading %zu elements per call\n",
	       SoapySDRDevice_getStreamMTU(s->dev, s->stream), s->read_elems);
	return 0;
}

/* Convert the signed/float samples delivered by SoapySDR to unsigned 8 bit. */
static void convert_to_u8(const sdr_ctx_t *s, const void *in, size_t elems,
			  uint8_t *out)
{
	size_t n = elems * 2;	/* interleaved I/Q */
	size_t i;

	switch (s->conv) {
	case CONV_CU8:
		memcpy(out, in, n);
		break;

	case CONV_CS8_TO_U8: {
		const int8_t *src = (const int8_t *)in;
		for (i = 0; i < n; i++)
			out[i] = (uint8_t)src[i] ^ 0x80u;
		break;
	}

	case CONV_CS16_TO_U8: {
		const int16_t *src = (const int16_t *)in;
		double scale = 127.0 / s->full_scale;
		for (i = 0; i < n; i++) {
			double v = src[i] * scale;
			long q;
			if (v > 127.0)
				v = 127.0;
			else if (v < -127.0)
				v = -127.0;
			q = lrint(v);
			out[i] = (uint8_t)((int8_t)q) ^ 0x80u;
		}
		break;
	}

	case CONV_CF32_TO_U8: {
		const float *src = (const float *)in;
		double scale = 127.0 / s->full_scale;
		for (i = 0; i < n; i++) {
			double v = src[i] * scale;
			long q;
			if (v > 127.0)
				v = 127.0;
			else if (v < -127.0)
				v = -127.0;
			q = lrint(v);
			out[i] = (uint8_t)((int8_t)q) ^ 0x80u;
		}
		break;
	}
	}
}

static void handle_command(sdr_ctx_t *s, uint8_t cmd, uint32_t param)
{
	switch (cmd) {
	case 0x01:	/* set center frequency */
		printf("set freq %u\n", param);
		if (SoapySDRDevice_setFrequency(s->dev, SOAPY_SDR_RX, s->channel,
						(double)param, NULL) != 0)
			fprintf(stderr, "rtl_tcp_soapy: setFrequency failed: %s\n",
				soapy_last_error());
		break;

	case 0x02:	/* set sample rate */
		printf("set sample rate %u\n", param);
		if (param > 0)
			set_sample_rate(s, (double)param);
		break;

	case 0x03:	/* set tuner gain mode: 0 = automatic, 1 = manual */
		printf("set gain mode %u\n", param);
		if (SoapySDRDevice_hasGainMode(s->dev, SOAPY_SDR_RX, s->channel) &&
		    SoapySDRDevice_setGainMode(s->dev, SOAPY_SDR_RX, s->channel,
					       param == 0) != 0)
			fprintf(stderr, "rtl_tcp_soapy: setGainMode failed: %s\n",
				soapy_last_error());
		break;

	case 0x04: {	/* set tuner gain, tenths of a dB */
		double db = param / 10.0;

		if (db < s->gain_min_db)
			db = s->gain_min_db;
		else if (db > s->gain_max_db)
			db = s->gain_max_db;
		printf("set gain %.1f dB\n", db);
		set_gain_db(s, db);
		break;
	}

	case 0x05:	/* set frequency correction in ppm */
		printf("set freq correction %d\n", (int32_t)param);
		if (SoapySDRDevice_setFrequencyCorrection(s->dev, SOAPY_SDR_RX,
							  s->channel,
							  (double)(int32_t)param) != 0)
			fprintf(stderr, "rtl_tcp_soapy: setFrequencyCorrection failed: %s\n",
				soapy_last_error());
		break;

	case 0x06: {	/* set IF stage gain */
		uint32_t stage = param >> 16;
		int16_t gain = (int16_t)(param & 0xffff);
		char **gains;
		size_t n = 0;
		printf("set if stage %u gain %.1f dB\n", stage, gain / 10.0);
		gains = SoapySDRDevice_listGains(s->dev, SOAPY_SDR_RX, s->channel, &n);
		if (gains && stage < n &&
		    SoapySDRDevice_setGainElement(s->dev, SOAPY_SDR_RX, s->channel,
						  gains[stage], gain / 10.0) != 0)
			fprintf(stderr, "rtl_tcp_soapy: setGainElement failed: %s\n",
				soapy_last_error());
		if (gains)
			SoapySDRStrings_clear(&gains, n);
		break;
	}

	case 0x07:	/* set test mode */
		printf("set test mode %u (ignored)\n", param);
		break;

	case 0x08:	/* set RTL2832 AGC mode */
		printf("set agc mode %u\n", param);
		if (SoapySDRDevice_hasGainMode(s->dev, SOAPY_SDR_RX, s->channel) &&
		    SoapySDRDevice_setGainMode(s->dev, SOAPY_SDR_RX, s->channel,
					       param != 0) != 0)
			fprintf(stderr, "rtl_tcp_soapy: setGainMode failed: %s\n",
				soapy_last_error());
		break;

	case 0x09:	/* set direct sampling */
		printf("set direct sampling %u (ignored)\n", param);
		break;

	case 0x0a:	/* set offset tuning */
		printf("set offset tuning %u (ignored)\n", param);
		break;

	case 0x0b:	/* set RTL xtal */
		printf("set rtl xtal %u (ignored)\n", param);
		break;

	case 0x0c:	/* set tuner xtal */
		printf("set tuner xtal %u (ignored)\n", param);
		break;

	case 0x0d:	/* set tuner gain by index into the spoofed table */
		if (param < (uint32_t)s->gain_count) {
			printf("set tuner gain by index %u (%.1f dB)\n",
			       param, s->gain_table[param] / 10.0);
			set_gain_db(s, s->gain_table[param] / 10.0);
		} else {
			printf("set tuner gain by index %u (out of range)\n", param);
		}
		break;

	case 0x0e:	/* set bias tee */
		printf("set bias tee %u\n", param);
		SoapySDRDevice_writeSetting(s->dev, "biastee", param ? "true" : "false");
		SoapySDRDevice_writeSetting(s->dev, "bias_tee", param ? "true" : "false");
		break;

	default:
		printf("unknown command 0x%02x (param 0x%08x)\n", cmd, param);
		break;
	}
}

/*
 * Read any pending 5 byte command packets from the client without blocking and
 * dispatch them.  Returns -1 when the client has disconnected.
 */
static int process_commands(sdr_ctx_t *s, int fd, uint8_t *buf, size_t *buflen)
{
	uint8_t tmp[256];

	for (;;) {
		ssize_t n = recv(fd, tmp, sizeof(tmp), MSG_DONTWAIT);
		size_t i;

		if (n > 0) {
			for (i = 0; i < (size_t)n; i++) {
				buf[(*buflen)++] = tmp[i];
				if (*buflen == 5) {
					uint32_t param =
						((uint32_t)buf[1] << 24) |
						((uint32_t)buf[2] << 16) |
						((uint32_t)buf[3] << 8) |
						(uint32_t)buf[4];
					handle_command(s, buf[0], param);
					*buflen = 0;
				}
			}
			continue;
		}
		if (n < 0 && errno == EINTR)
			continue;
		if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
			return 0;
		return -1;	/* orderly shutdown or error */
	}
}

/* Stream until the client goes away or a command changes the configuration. */
static void serve_client(sdr_ctx_t *s, int fd)
{
	uint8_t *inbuf, *ubuf, *outbuf;
	void *buffs[1];
	uint8_t cmd_buf[5];
	size_t cmd_len = 0;
	unsigned long overflows = 0;

	inbuf = xmalloc(s->read_elems * s->in_elem_bytes);
	ubuf = xmalloc(s->read_elems * 2);
	outbuf = xmalloc(s->read_elems * 2);
	buffs[0] = inbuf;

	/* start every client from the configuration selected on the command line */
	printf("new client: restoring initial device configuration\n");
	apply_config(s);

	if (SoapySDRDevice_activateStream(s->dev, s->stream, 0, 0, 0) != 0)
		fprintf(stderr, "rtl_tcp_soapy: activateStream failed: %s\n",
			soapy_last_error());

	while (!g_do_exit) {
		int flags = 0;
		long long time_ns = 0;
		int n;

		if (process_commands(s, fd, cmd_buf, &cmd_len) != 0)
			break;

		n = SoapySDRDevice_readStream(s->dev, s->stream, buffs,
					      s->read_elems, &flags, &time_ns,
					      READ_TIMEOUT_US);
		if (n == SOAPY_SDR_TIMEOUT)
			continue;
		if (n == SOAPY_SDR_OVERFLOW) {
			if ((++overflows & 0xff) == 1)
				fprintf(stderr, "rtl_tcp_soapy: stream overflow\n");
			continue;
		}
		if (n < 0) {
			fprintf(stderr, "rtl_tcp_soapy: readStream failed: %s\n",
				soapy_last_error());
			break;
		}
		if (n == 0)
			continue;

		convert_to_u8(s, inbuf, (size_t)n, ubuf);
		if (s->decim.factor > 1) {
			size_t m = decimator_process(&s->decim, ubuf, (size_t)n,
						     outbuf);
			if (send_all(fd, outbuf, m * 2) != 0)
				break;
		} else {
			if (send_all(fd, ubuf, (size_t)n * 2) != 0)
				break;
		}
	}

	SoapySDRDevice_deactivateStream(s->dev, s->stream, 0, 0);
	free(inbuf);
	free(ubuf);
	free(outbuf);
}

int main(int argc, char **argv)
{
	char addr[256] = DEFAULT_ADDR;
	char port[64] = DEFAULT_PORT;
	char *args = NULL;
	size_t channel = 0;
	double frequency = DEFAULT_FREQ_HZ;
	double sample_rate = DEFAULT_RATE_HZ;
	double bandwidth = 0.0;
	int have_bandwidth = 0;
	double gain = 0.0;
	int have_gain = 0;
	int biastee = 0;
	int tuner_type = DEFAULT_TUNER_TYPE;
	int decim_enabled = 1;
	double min_hw_rate = DEFAULT_MIN_HW_RATE;
	sdr_ctx_t sdr;
	struct addrinfo hints;
	struct addrinfo *ai_head = NULL;
	struct addrinfo *ai;
	int listen_fd = -1;
	int i;
	int ret;

	/* keep the log useful when stdout is redirected to a file or pipe */
	setvbuf(stdout, NULL, _IOLBF, 0);

	for (i = 1; i < argc; i++) {
		const char *a = argv[i];
		const char *val = NULL;

		if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
			usage(argv[0]);
		} else if (!strcmp(a, "--args")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: --args needs a value\n");
				return 1;
			}
			args = argv[i];
		} else if (!strncmp(a, "--args=", 7)) {
			args = (char *)a + 7;
		} else if (!strcmp(a, "--listen")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: --listen needs a value\n");
				return 1;
			}
			val = argv[i];
		} else if (!strncmp(a, "--listen=", 9)) {
			val = a + 9;
		} else if (!strcmp(a, "-a") || !strcmp(a, "--addr")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: %s needs a value\n", a);
				return 1;
			}
			snprintf(addr, sizeof(addr), "%s", argv[i]);
		} else if (!strncmp(a, "--addr=", 7)) {
			snprintf(addr, sizeof(addr), "%s", a + 7);
		} else if (!strcmp(a, "-p") || !strcmp(a, "--port")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: %s needs a value\n", a);
				return 1;
			}
			snprintf(port, sizeof(port), "%s", argv[i]);
		} else if (!strncmp(a, "--port=", 7)) {
			snprintf(port, sizeof(port), "%s", a + 7);
		} else if (!strcmp(a, "-f") || !strcmp(a, "--freq")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: %s needs a value\n", a);
				return 1;
			}
			frequency = parse_hz(argv[i]);
		} else if (!strncmp(a, "--freq=", 7)) {
			frequency = parse_hz(a + 7);
		} else if (!strcmp(a, "-s") || !strcmp(a, "--samplerate")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: %s needs a value\n", a);
				return 1;
			}
			sample_rate = parse_hz(argv[i]);
		} else if (!strncmp(a, "--samplerate=", 13)) {
			sample_rate = parse_hz(a + 13);
		} else if (!strcmp(a, "-b") || !strcmp(a, "--bandwidth")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: %s needs a value\n", a);
				return 1;
			}
			bandwidth = parse_hz(argv[i]);
			have_bandwidth = 1;
		} else if (!strncmp(a, "--bandwidth=", 12)) {
			bandwidth = parse_hz(a + 12);
			have_bandwidth = 1;
		} else if (!strcmp(a, "-g") || !strcmp(a, "--gain")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: %s needs a value\n", a);
				return 1;
			}
			gain = atof(argv[i]);
			have_gain = 1;
		} else if (!strncmp(a, "--gain=", 7)) {
			gain = atof(a + 7);
			have_gain = 1;
		} else if (!strcmp(a, "-c") || !strcmp(a, "--channel")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: %s needs a value\n", a);
				return 1;
			}
			channel = (size_t)strtoul(argv[i], NULL, 0);
		} else if (!strncmp(a, "--channel=", 10)) {
			channel = (size_t)strtoul(a + 10, NULL, 0);
		} else if (!strcmp(a, "-T") || !strcmp(a, "--bias-tee")) {
			biastee = 1;
		} else if (!strcmp(a, "-D") || !strcmp(a, "--direct-sampling")) {
			/* accepted for rtl_tcp compatibility, not applicable here */
		} else if (!strcmp(a, "--tuner-type")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: %s needs a value\n", a);
				return 1;
			}
			tuner_type = atoi(argv[i]);
		} else if (!strncmp(a, "--tuner-type=", 13)) {
			tuner_type = atoi(a + 13);
		} else if (!strcmp(a, "--no-decimate")) {
			decim_enabled = 0;
		} else if (!strcmp(a, "--min-hw-rate")) {
			if (++i >= argc) {
				fprintf(stderr, "rtl_tcp_soapy: %s needs a value\n", a);
				return 1;
			}
			min_hw_rate = parse_hz(argv[i]);
		} else if (!strncmp(a, "--min-hw-rate=", 14)) {
			min_hw_rate = parse_hz(a + 14);
		} else {
			fprintf(stderr, "rtl_tcp_soapy: unknown option '%s'\n", a);
			usage(argv[0]);
		}

		/* Handle the "host:port" syntax of --listen. */
		if (val)
			parse_listen(val, addr, sizeof(addr), port, sizeof(port));
	}

	if (sample_rate <= 0.0 || frequency <= 0.0) {
		fprintf(stderr, "rtl_tcp_soapy: invalid frequency or sample rate\n");
		return 1;
	}

	memset(&sdr, 0, sizeof(sdr));
	sdr.channel = channel;
	sdr.decim_enabled = decim_enabled;
	sdr.min_hw_rate = min_hw_rate;
	sdr.decim.factor = 1;
	sdr.init.freq = frequency;
	sdr.init.rate = sample_rate;
	sdr.init.bandwidth = bandwidth;
	sdr.init.have_bandwidth = have_bandwidth;
	sdr.init.gain = gain;
	sdr.init.have_gain = have_gain;
	sdr.init.biastee = biastee;

	sdr.dev = SoapySDRDevice_makeStrArgs(args ? args : "");
	if (!sdr.dev) {
		if (args)
			fprintf(stderr, "rtl_tcp_soapy: failed to open SoapySDR device "
				"with args \"%s\": %s\n", args, soapy_last_error());
		else
			fprintf(stderr, "rtl_tcp_soapy: failed to open SoapySDR device: %s\n",
				soapy_last_error());
		return 1;
	}

	{
		char *drv = SoapySDRDevice_getDriverKey(sdr.dev);
		char *hw = SoapySDRDevice_getHardwareKey(sdr.dev);
		printf("opened SoapySDR device: driver=%s hardware=%s channel=%zu\n",
		       drv ? drv : "?", hw ? hw : "?", channel);
		SoapySDR_free(drv);
		SoapySDR_free(hw);
	}

	apply_config(&sdr);
	build_gain_table(&sdr);

	if (setup_stream(&sdr) != 0) {
		decimator_free(&sdr.decim);
		free(sdr.gain_table);
		SoapySDRDevice_unmake(sdr.dev);
		return 1;
	}

	signal(SIGPIPE, SIG_IGN);
	signal(SIGINT, sighandler);
	signal(SIGTERM, sighandler);

	memset(&hints, 0, sizeof(hints));
	hints.ai_flags = AI_PASSIVE;
	hints.ai_family = PF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	ret = getaddrinfo(addr[0] ? addr : NULL, port[0] ? port : NULL,
			  &hints, &ai_head);
	if (ret != 0) {
		fprintf(stderr, "rtl_tcp_soapy: getaddrinfo(%s:%s) failed: %s\n",
			addr, port, gai_strerror(ret));
		SoapySDRDevice_closeStream(sdr.dev, sdr.stream);
		SoapySDRDevice_unmake(sdr.dev);
		decimator_free(&sdr.decim);
		free(sdr.gain_table);
		return 1;
	}

	for (ai = ai_head; ai != NULL; ai = ai->ai_next) {
		int one = 1;
		listen_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
		if (listen_fd < 0)
			continue;
		setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
		if (bind(listen_fd, ai->ai_addr, ai->ai_addrlen) == 0)
			break;
		close(listen_fd);
		listen_fd = -1;
	}
	freeaddrinfo(ai_head);

	if (listen_fd < 0) {
		fprintf(stderr, "rtl_tcp_soapy: failed to bind %s:%s: %s\n",
			addr, port, strerror(errno));
		SoapySDRDevice_closeStream(sdr.dev, sdr.stream);
		SoapySDRDevice_unmake(sdr.dev);
		decimator_free(&sdr.decim);
		free(sdr.gain_table);
		return 1;
	}

	if (listen(listen_fd, 1) != 0)
		fprintf(stderr, "rtl_tcp_soapy: listen failed: %s\n", strerror(errno));

	printf("listening on %s:%s\n", addr[0] ? addr : "*", port[0] ? port : "*");
	printf("connect rtl_tcp clients to rtl_tcp=%s:%s\n",
	       addr[0] ? addr : "127.0.0.1", port[0] ? port : DEFAULT_PORT);

	while (!g_do_exit) {
		struct sockaddr_storage remote;
		socklen_t rlen = sizeof(remote);
		struct timeval tv;
		fd_set readfds;
		int fd;
		dongle_info_t info;
		char host[NI_MAXHOST];
		char serv[NI_MAXSERV];
		int one = 1;

		FD_ZERO(&readfds);
		FD_SET(listen_fd, &readfds);
		tv.tv_sec = 1;
		tv.tv_usec = 0;

		ret = select(listen_fd + 1, &readfds, NULL, NULL, &tv);
		if (g_do_exit)
			break;
		if (ret <= 0)
			continue;

		fd = accept(listen_fd, (struct sockaddr *)&remote, &rlen);
		if (fd < 0) {
			if (errno != EINTR)
				fprintf(stderr, "rtl_tcp_soapy: accept failed: %s\n",
					strerror(errno));
			continue;
		}

		g_client_fd = fd;
		setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
		{
			struct timeval snd = { 5, 0 };
			setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd, sizeof(snd));
		}

		if (getnameinfo((struct sockaddr *)&remote, rlen, host, sizeof(host),
				serv, sizeof(serv),
				NI_NUMERICHOST | NI_NUMERICSERV) == 0)
			printf("client accepted! %s %s\n", host, serv);
		else
			printf("client accepted!\n");

		memset(&info, 0, sizeof(info));
		memcpy(info.magic, "RTL0", 4);
		info.tuner_type = htonl((uint32_t)tuner_type);
		info.tuner_gain_count = htonl((uint32_t)sdr.gain_count);
		if (send_all(fd, &info, sizeof(info)) != 0) {
			fprintf(stderr, "rtl_tcp_soapy: failed to send dongle info\n");
			close(fd);
			g_client_fd = -1;
			continue;
		}

		serve_client(&sdr, fd);

		close(fd);
		g_client_fd = -1;
		printf("client disconnected\n");
	}

	printf("bye!\n");
	close(listen_fd);
	SoapySDRDevice_closeStream(sdr.dev, sdr.stream);
	SoapySDRDevice_unmake(sdr.dev);
	decimator_free(&sdr.decim);
	free(sdr.gain_table);
	return 0;
}
