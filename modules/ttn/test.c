/**
 * Convert a given key as char array to an array of hex
 * keys are defined as arrays of char in ttn
 * eg const char *devAddr = "26011CFB";
 * should return {0x26, 0x01, 0x1C, 0xFB}
 *
 * gcc -o test test.c && ./test
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void char2int(const char *addr, uint8_t length, char *result) {
  uint8_t number;
  for(uint8_t i=0; i<length; i++) {
    char hex[2];
    memcpy(hex, &addr[i*2], 2);
    number = (uint8_t)strtol(hex, NULL, 16);
    result[i] = number;
    printf("%c%c: 0x%.2X (%u)\n", addr[i*2], addr[i*2+1], number, number);
  }

}

int main(void) {
  printf("Hello\n");
  const char *dev_addr = "26011CFB";
  uint8_t length = 4;
  const char dev_addr_should[4] = {0x26, 0x01, 0x1C, 0xFB};
  char dev_addr_generated[length];
  char2int(dev_addr, length, dev_addr_generated);
  uint8_t errors = 0;
  for(uint8_t i=0; i<length; i++) {
    if(dev_addr_generated[i] != dev_addr_should[i]) {
      errors++;
    }
  }
  if (errors) {
    printf("Errors occured: %u\n", errors);
  } else {
    printf("All good.\n");
  }
  return 0;
}

