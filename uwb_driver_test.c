/*
* Copyright  2022 NXP
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

#define SRXXX_MAGIC 0xEA
#define SRXXX_SET_PWR _IOW(SRXXX_MAGIC, 0x01, uint32_t)
#define SRXXX_SET_FWD _IOW(SRXXX_MAGIC, 0x02, uint32_t)

#define UWBSTATUS_SUCCESS 0x00
#define UWBSTATUS_FAILED 0x02
#define PH_TMLUWB_RESET_VALUE (0x00)
#define UCI_MAX_DATA_LEN 4200
#define NORMAL_MODE_HEADER_LEN    4
#define PHHBCI_MODE_LEN_MSB_OFFSET (3U)
#define PHHBCI_MODE_LEN_LSB_OFFSET (2U)

int fd = -1;
uint8_t autoload_info_cmd[] = {0x01, 0x36, 0x00,0x00};
uint8_t ack_query_cmd[] = {0x04, 0x01, 0x00,0x00};
uint8_t qry_chip_id_cmd[] = {0x01, 0x31, 0x00,0x00};

int uwb_spi_read(int pDevHandle, uint8_t* pBuffer, int nNbBytesToRead) {
    int ret_Read, i;
    uint16_t totalBtyesToRead = 0;

    if (pDevHandle < 0) {
        printf("_spi_read() error handle");
        return -1;
    }
    totalBtyesToRead = nNbBytesToRead;
    ret_Read = read(pDevHandle, pBuffer, totalBtyesToRead);
    if (ret_Read < 0) {
        printf("_spi_read() error: %d", ret_Read);
        return -1;
    }
    printf("Read value ret_Read = %d  value = ", ret_Read);
    for (i = 0; i < ret_Read; i++) {
        printf("  %02X ", pBuffer[i]);
    }
    printf("\n");
    return ret_Read;
}

int uwb_spi_write(int pDevHandle, uint8_t* pBuffer,
                       int nNbBytesToWrite) {
  int ret, i;
  int numWrote = 0;

  if (pDevHandle < 0) {
    printf("_spi_write() device is invalid");
    return -1;
  }
  printf("Write value = ");
  for (i = 0; i < nNbBytesToWrite; i++) {
      printf("  %02X ", pBuffer[i]);
  }
  printf("\n");

  ret = write(pDevHandle, pBuffer, nNbBytesToWrite);
  if (ret > 0) {
    printf("_spi_write()_1 ret : %x \n", ret);
    numWrote = ret;
  } else {
    printf("_spi_write()_1 failed : %d", ret);
    return -1;
  }
  return numWrote;
}

int main()
{
    uint8_t total_bytes_to_read_msb = 0;
    uint16_t total_bytes_to_read = 0;
    uint8_t temp[UCI_MAX_DATA_LEN];

    printf("\nOpening Driver\n");
    fd = open("/dev/sr1xx", O_RDWR);
    if(fd < 0) {
        printf("Cannot open device file...\n");
        return 0;
    }
    ioctl(fd, SRXXX_SET_PWR, 0);
    usleep(1000);
    ioctl(fd, SRXXX_SET_PWR, 1);
    usleep(1000);

    int ret = -1;

    ioctl(fd, SRXXX_SET_FWD, 1);
    uwb_spi_write(fd, autoload_info_cmd, NORMAL_MODE_HEADER_LEN);
    ret = uwb_spi_read(fd, temp, NORMAL_MODE_HEADER_LEN);

    total_bytes_to_read_msb = (temp[PHHBCI_MODE_LEN_MSB_OFFSET] & 0x0F);
    total_bytes_to_read = (uint16_t)(temp[PHHBCI_MODE_LEN_LSB_OFFSET] | (total_bytes_to_read_msb << 8));
    usleep(1);

    uwb_spi_write(fd, ack_query_cmd, NORMAL_MODE_HEADER_LEN);
    ret = uwb_spi_read(fd, temp, total_bytes_to_read);

    usleep(1);
    uwb_spi_write(fd, qry_chip_id_cmd, NORMAL_MODE_HEADER_LEN);
    ret = uwb_spi_read(fd, temp, NORMAL_MODE_HEADER_LEN);

    total_bytes_to_read_msb = (temp[PHHBCI_MODE_LEN_MSB_OFFSET] & 0x0F);
    total_bytes_to_read = (uint16_t)(temp[PHHBCI_MODE_LEN_LSB_OFFSET] | (total_bytes_to_read_msb << 8));
    usleep(1);

    uwb_spi_write(fd, ack_query_cmd, NORMAL_MODE_HEADER_LEN);
    ret = uwb_spi_read(fd, temp, total_bytes_to_read);

    ioctl(fd, SRXXX_SET_FWD, 0);

    if (fd != -1)
    {
        close(fd);
    }
}
