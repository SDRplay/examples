/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
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
 *
 * ************************************** THIS IS A FORK ******************* ORIGINAL COPYRIGHT SEE ABOVE.
 *
 *  SDRPlayPorts
 *  Ports of some parts of rtl-sdr for the SDRPlay (original: git://git.osmocom.org/rtl-sdr.git /)
 *  2016: Fork by HB9FXQ (Frank Werner-Krippendorf, mail@hb9fxq.ch)
 *
 *  Code changes I've done:
 *  - removed rtl_sdr related calls and replaced them with mir_* to work with the SDRPlay
 *  - removed various local variables
 *
 *  This program is free software: you can redistribute it and/or modify
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
 *
 *
 *
 */

#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32

#include <unistd.h>
#include "mirsdrapi-rsp.h"

#else
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "mir_sdr.h"
#endif

#define DEFAULT_SAMPLE_RATE        2048000
#define DEFAULT_LNA                0;
#define DEFAULT_GAIN_REDUCTION  0;
#define DEFAULT_GAIN            40;
#define DEFAULT_FREQUENCY       100000000;
#define DEFAULT_RESULT_BITS     8; // more compatible with RTL_SDR

static int do_exit = 0;

short *ibuf;
short *qbuf;
unsigned int firstSample;
int samplesPerPacket, grChanged, fsChanged, rfChanged;
int resultBits = DEFAULT_RESULT_BITS;

void adjust_bw(int bwHz, mir_sdr_Bw_MHzT *ptr);

void adjust_if(int ifFreq, mir_sdr_If_kHzT *ptr);

void adjust_result_bits(int bits, int *ptrResultBits);

double atofs(char *s)
/* standard suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	last = s[len - 1];
	s[len - 1] = '\0';
	switch (last) {
		case 'g':
		case 'G':
			suff *= 1e3;
		case 'm':
		case 'M':
			suff *= 1e3;
		case 'k':
		case 'K':
			suff *= 1e3;
			suff *= atof(s);
			s[len - 1] = last;
			return suff;
	}
	s[len - 1] = last;
	return atof(s);
}

void usage(void) {
	fprintf(stderr,
			"play_sdr, an I/Q recorder for SDRplay RSP receivers\n\n"
					"Usage:\t -f frequency_to_tune_to (Hz)\n"
					"\t[-s samplerate (default: 2048000 Hz)]\n"
					"\t[-b Band Width in Hz (default: 1536) possible values: 200 300 600 1536 5000 6000 7000 8000]\n"
					"\t[-i IF in kHz (default: 0 (=Zero IF)) possible values: 0 450 1620 2048]\n"
					"\t[-g gain (default: 50)]\n"
					"\t[-r enable gain reduction (default: 0, disabled)]\n"
					"\t[-l RSP LNA enable (default: 0, disabled)]\n"
					"\t[-y Flipcomplex I-Q => Q-I (default: 0, disabled) 1 = enabled\n"
					"\t[-x Result I/Q bit resolution (uint8 / short) (default: 8, possible values: 8 16)]\n"
					"\t[-v Verbose mode, prints debug information. Default 0, 1 = enabled\n"
					"\tfilename (a '-' dumps samples to stdout)\n\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
    if (CTRL_C_EVENT == signum) {
        fprintf(stderr, "Signal caught, exiting!\n");
        do_exit = 1;
        mir_sdr_Uninit();
        return TRUE;
    }
    return FALSE;
}
#else

static void sighandler(int signum) {
	fprintf(stderr, "Signal (%d) caught, exiting!\n", signum);
	do_exit = 1;
	mir_sdr_Uninit();
}

#endif

int main(int argc, char **argv) {
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	int bufferSize;
	mir_sdr_ErrT r;
	int opt;
	int gain = DEFAULT_GAIN;
	int flipcomplex = 0;
	int verbose = 0;
	FILE *file;

	uint8_t *buffer8;
	short *buffer16;

	uint32_t frequency = DEFAULT_FREQUENCY;
	uint32_t samp_rate = DEFAULT_SAMPLE_RATE;
	int rspLNA = DEFAULT_LNA;
	int i, j;
	mir_sdr_Bw_MHzT bandwidth = mir_sdr_BW_1_536;
	mir_sdr_If_kHzT ifKhz = mir_sdr_IF_Zero;

	while ((opt = getopt(argc, argv, "f:g:s:n:l:b:i:x:y:v:")) != -1) {
		switch (opt) {
			case 'f':
				frequency = (uint32_t) atofs(optarg);
				break;
			case 'g':
				gain = (int) atof(optarg);
				break;
			case 's':
				samp_rate = (uint32_t) atofs(optarg);
				break;
			case 'l':
				rspLNA = atoi(optarg);
				break;
			case 'b':
				adjust_bw(atoi(optarg), &bandwidth);
				break;
			case 'i':
				adjust_if(atoi(optarg), &ifKhz);
				break;
			case 'x':
				adjust_result_bits(atoi(optarg), &resultBits);
				break;
			case 'y':
				flipcomplex = atoi(optarg);
				break;
			case 'v':
				verbose = atoi(optarg);
				break;
			default:
				usage();
				break;
		}
	}

	/* FIXME, validate inputs, as suggested by Andy:
     * """You also might want to do some checking of sample rate and IF bandiwith and IF mode as only certain combinations are valid states.
     * Although you will get an error condition from the API if it detects an invalid case."""
     *
    */

	if (argc <= optind) {
		usage();
	} else {
		filename = argv[optind];
	}

	if (verbose == 1) {
		fprintf(stderr, "[DEBUG] *************** play_sdr16 init summary *********************\n");
		fprintf(stderr, "[DEBUG] LNA: %d\n", rspLNA);
		fprintf(stderr, "[DEBUG] samp_rate: %d\n", samp_rate);
		fprintf(stderr, "[DEBUG] gain: %d\n", gain);
		fprintf(stderr, "[DEBUG] frequency: [Hz] %d / [MHz] %f\n", frequency, frequency / 1e6);
		fprintf(stderr, "[DEBUG] bandwidth: [kHz] %d\n", bandwidth);
		fprintf(stderr, "[DEBUG] IF: %d\n", ifKhz);
		fprintf(stderr, "[DEBUG] Result I/Q bit resolution (bit): %d\n", resultBits);
		fprintf(stderr, "[DEBUG] *************************************************************\n");
	}


	r = mir_sdr_Init(40, 2.0, 100.00, mir_sdr_BW_1_536, mir_sdr_IF_Zero,
					 &samplesPerPacket);

	if (r != mir_sdr_Success) {
		fprintf(stderr, "Failed to open SDRplay RSP device.\n");
		exit(1);
	}

	mir_sdr_Uninit();

#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	if (strcmp(filename, "-") == 0) { /* Write samples to stdout */
		file = stdout;
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
	} else {
		file = fopen(filename, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			goto out;
		}
	}


	mir_sdr_SetParam(201, 1);
	mir_sdr_SetParam(202, rspLNA == 1 ? 0 : 1);


	r = mir_sdr_Init(gain, (samp_rate / 1e6), (frequency / 1e6),
					 bandwidth, ifKhz, &samplesPerPacket);


	bufferSize = (samplesPerPacket * 2);

	if (resultBits == 8) {
		buffer8 = malloc(bufferSize * sizeof(uint8_t));
	}
	else {
		buffer16 = malloc(bufferSize * sizeof(short));
	}


	if (r != mir_sdr_Success) {
		fprintf(stderr, "Failed to start SDRplay RSP device.\n");
		exit(1);
	}

	mir_sdr_SetDcMode(4, 0);
	mir_sdr_SetDcTrackTime(63);

	ibuf = malloc(samplesPerPacket * sizeof(short));
	qbuf = malloc(samplesPerPacket * sizeof(short));

	fprintf(stderr, "Writing samples...\n");

	while (!do_exit) {
		r = mir_sdr_ReadPacket(ibuf, qbuf, &firstSample, &grChanged, &rfChanged,
							   &fsChanged);

		if (r != mir_sdr_Success) {
			fprintf(stderr, "WARNING: ReadPacket failed.\n");
			break;
		}

		j = 0;
		for (i = 0; i < samplesPerPacket; i++) {
			if (resultBits == 8) {
				if (flipcomplex == 0) {
					buffer8[j++] = (unsigned char) (ibuf[i] >> 8);
					buffer8[j++] = (unsigned char) (qbuf[i] >> 8);
				} else {
					buffer8[j++] = (unsigned char) (qbuf[i] >> 8);
					buffer8[j++] = (unsigned char) (ibuf[i] >> 8);
				}
			}
			else {
				if (flipcomplex == 0) {
					buffer16[j++] = ibuf[i];
					buffer16[j++] = qbuf[i];
				} else {
					buffer16[j++] = qbuf[i];
					buffer16[j++] = ibuf[i];
				}
			}
		}

		if (resultBits == 8) {
			if (fwrite(buffer8, sizeof(uint8_t), bufferSize, file) != (size_t) bufferSize) {
				fprintf(stderr, "Short write, samples lost, exiting!\n");
				break;
			}
		}
		else {
			if (fwrite(buffer16, sizeof(short), samplesPerPacket * 2, file) != (size_t) bufferSize) {
				fprintf(stderr, "Short write, samples lost, exiting!\n");
				break;
			}
		}
	}


	mir_sdr_Uninit();

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	if (file != stdout)
		fclose(file);


	if (resultBits == 8) {
		free(buffer8);
	}
	else {
		free(buffer16);
	}

	out:
	return r >= 0 ? r : -r;
}

void adjust_result_bits(int bits, int *ptrResultBits) {
	switch (bits) {
		case 8:
		case 16:
			*ptrResultBits = bits;
			return;
	}

	fprintf(stderr, "Invalid result I/Q resolution (-x) !\n");
	usage();
}

void adjust_if(int ifFreq, mir_sdr_If_kHzT *ptrIFKhz) {
	switch (ifFreq) {
		case 0:
		case 450:
		case 1620:
		case 2048:
			*ptrIFKhz = ifFreq;
			return;
	}

	fprintf(stderr, "Invalid IF frequency (-i) !\n");
	usage();

}

void adjust_bw(int bwHz, mir_sdr_Bw_MHzT *ptrBw) {

	switch (bwHz) {
		case 200:
		case 300:
		case 600:
		case 1536:
		case 5000:
		case 6000:
		case 8000:
			*ptrBw = bwHz;
			return;
	}

	fprintf(stderr, "Invalid Band Width (-b) !\n");
	usage();
}
