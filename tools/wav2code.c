// wav2code.c
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// WAVE file header format
typedef struct __attribute__((packed))
{
    char riff[4];              // RIFF string
    uint32_t overall_size;     // overall size of file in bytes
    char wave[4];              // WAVE string
    char fmt_chunk_marker[4];  // fmt string with trailing null char
    uint32_t length_of_fmt;    // length of the format data
    uint16_t format_type;      // format type. 1-PCM, 3- IEEE float, 6 - 8bit A law, 7 - 8bit mu law
    uint16_t channels;         // no.of channels
    uint32_t sample_rate;      // sampling rate (blocks per second)
    uint32_t byterate;         // SampleRate * NumChannels * BitsPerSample/8
    uint16_t block_align;      // NumChannels * BitsPerSample/8
    uint16_t bits_per_sample;  // bits per sample, 8- 8bits, 16- 16 bits etc
    char data_chunk_header[4]; // DATA string or FLLR string
    uint32_t data_size;        // NumSamples * NumChannels * BitsPerSample/8 - size of the next chunk that will be read
} WAV_Header_t;

/**
 * @brief  Pack 4 samples from the source buffer into the destination buffer
 * @param  dst - pointer to the destination buffer
 * @param  src - pointer to the source buffer
 * @return None
 * @note
 *  Sample packing:
 *  Sample: [0][1][2][3]
 *  Bytes:  [00000000][0011 1111][1111 2222][2222 2233][3333 3333]
 */
static inline void pack4(uint8_t *dst, const int16_t *src)
{
    dst[0] = (src[0] >> 2) & 0xFF;
    dst[1] = ((src[0] & 0x03) << 6) | (src[1] >> 4);
    dst[2] = ((src[1] & 0x0F) << 4) | (src[2] >> 6);
    dst[3] = ((src[2] & 0x3F) << 2) | (src[3] >> 8);
    dst[4] = src[3] & 0xFF;
}

int main(int argc, char **argv)
{

    if (argc < 2)
    {
        printf("Usage: %s <filename.wav>\n", argv[0]);
        exit(1);
    }

    const char *const filename = argv[1];

    FILE *f = fopen(filename, "rb");
    if (f == NULL)
    {
        printf("Error opening file: %s\n", filename);
        exit(1);
    }

    WAV_Header_t header;
    const int read = fread(&header, sizeof(header), 1, f);
    if (!read)
    {
        printf("Error reading file header\n");
        fclose(f);
        exit(1);
    }

    printf("// RIFF: %.4s\n", header.riff);
    printf("// Overall size: %d bytes\n", header.overall_size);
    printf("// WAVE: %.4s\n", header.wave);
    printf("// Format chunk marker: %.4s\n", header.fmt_chunk_marker);
    printf("// Length of format: %d bytes\n", header.length_of_fmt);
    printf("// Format type: %d\n", header.format_type);
    printf("// Channels: %d\n", header.channels);
    printf("// Sample rate: %d Hz\n", header.sample_rate);
    printf("// Byte rate: %d bytes/sec\n", header.byterate);
    printf("// Block align: %d bytes\n", header.block_align);
    printf("// Bits per sample: %d bits\n", header.bits_per_sample);
    printf("// Data chunk header: %.4s\n", header.data_chunk_header);
    printf("// Data size: %d bytes\n", header.data_size);

    const size_t size_of_each_sample = (header.bits_per_sample / 8) * header.channels;
    size_t num_samples = header.data_size / size_of_each_sample;
    num_samples = num_samples + (num_samples % 4); // Round up to the next multiple of 4

    const size_t padded_data_size = num_samples * size_of_each_sample;

    uint16_t *data_buffer = calloc(padded_data_size, 1);

    if (!fread(data_buffer, header.data_size, 1, f))
    {
        printf("Error reading sample data\n");
        free(data_buffer);
        fclose(f);
        exit(1);
    }

    if (header.bits_per_sample != 16 || header.format_type != 1)
    {
        printf("Only 16-bit PCM samples are supported\n");
        free(data_buffer);
        fclose(f);
        exit(1);
    }

    printf("#include <stdint.h>\n");
    printf("#include <assert.h>\n");
    printf("\n\nstatic const uint8_t samples[] = {\n");
    size_t cols = 0;
    int16_t sample_buffer[4] = {0};
    uint8_t packed_buffer[5] = {0};
    for (size_t j = 0; j < header.channels; j++)
    {
        for (size_t i = 0; i < num_samples; i += 4)
        {
            for (size_t k = 0; k < 4; k++)
            {
                sample_buffer[k] = ((int16_t *)data_buffer)[(i + k) * header.channels + j];
                sample_buffer[k] += 0x8000; // Convert to unsigned
                sample_buffer[k] >>= 6;     // convert to 10 bit
            }

            pack4(packed_buffer, sample_buffer);
            printf("%d, %d, %d, %d, %d, ",
                   (int)packed_buffer[0],
                   (int)packed_buffer[1],
                   (int)packed_buffer[2],
                   (int)packed_buffer[3],
                   (int)packed_buffer[4]);

            cols += 5;
            if (cols >= 16)
            {
                printf("\n");
                cols = 0;
            }
        }
    }
    printf("};\n");
    printf("static_assert(sizeof(samples) %% 5 == 0, \"Something went wrong, buffer size must be a multiple of 5 bytes\");\n");

    fclose(f);
    free(data_buffer);

    return 0;
}
