#include "serial.h"
#include <errno.h>
#include <stdio.h>
#include <cstdlib>

static Serial serial;
static char msg[256];
static float x, y, z;
static FILE *datafile;

void dry_run()
{
	for (int i = 0; i < 10; ++i) {
		serial.readLine(msg, sizeof(msg));
	}
}

int main()
{
    const char *filename = getenv("DATA_FILE");
	if (!filename) {
                datafile = stdout;
        } else {
                datafile = fopen(filename, "w+");
        	if (datafile == NULL) {
        		printf("Failed to open file %s, %s", filename, strerror(errno));
        		return EXIT_FAILURE;
        	}
                printf("Writing data to file %s\n", filename);
        }

	if (serial.openSerial()) {
		dry_run();
		while (1) {
			if (!serial.readLine(msg, sizeof(msg) - 1)) {
				fclose(datafile);
				return EXIT_FAILURE;
			}

			fprintf(datafile, "%s", msg);
                        fflush(datafile);
		}
	}

	fclose(datafile);
	serial.closeSerial();
	return EXIT_SUCCESS;
}
