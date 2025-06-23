#include <fcntl.h>
#include <linux/i2c.h>
#include <stdio.h>
#include "i2c.h"

#define USE_LOCKING 1
#ifdef USE_LOCKING
#define LOCK_MUTEX    do {pthread_mutex_lock(&bus->mutex);} while (0)
#define UNLOCK_MUTEX  do {pthread_mutex_unlock(&bus->mutex);} while (0)
#else
#define LOCK_MUTEX {}
#define UNLOCK_MUTEX {}
#endif 

#ifdef DEBUG_I2C_WACKO
#define DEBUG_INFO perror
#else
#define DEBUG_INFO(x) {}
#endif

struct i2c_t {
	int fd;
	pthread_mutex_t mutex;
};

i2c i2c_open(const char *filename) {
	i2c bus = (i2c)malloc(sizeof(struct i2c_t));

	// Open I2C file
	if((bus->fd = open(filename, O_RDWR)) < 0) {
		free(bus);
		return NULL;
	}

#ifdef USE_LOCKING
	// create bus mutex
	if(pthread_mutex_init(&bus->mutex, NULL)) {
		close(bus->fd);
		free(bus);
		return NULL;
	}
#endif

	return bus;
}

void i2c_write(i2c bus, uint8_t addr, uint8_t *buffer, uint8_t buffersize) 
{

	struct i2c_msg msgs[1];
	struct i2c_rdwr_ioctl_data msgset;
	msgs[0].addr = addr;
	msgs[0].flags = 0;
	msgs[0].len = buffersize;
	msgs[0].buf = buffer;

	msgset.msgs = msgs;
	msgset.nmsgs = 1;

	/* Lock mutex */
	LOCK_MUTEX;

	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in write8");
	}

	/* Unlock mutex */
	UNLOCK_MUTEX;
}

uint8_t i2c_reg8_read8(i2c bus, uint8_t i2caddr, uint8_t reg)
{
	uint8_t databuf[1];
	uint8_t addrbuf[1];
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data msgset;

	msgs[0].addr = i2caddr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = addrbuf;

	msgs[1].addr = i2caddr;
	msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
	msgs[1].len = 1;
	msgs[1].buf = databuf;

	msgset.msgs = msgs;
	msgset.nmsgs = 2;

	addrbuf[0] = reg;

	LOCK_MUTEX;
	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in i2c_read");
	}
	UNLOCK_MUTEX;

	return databuf[0];
}

uint16_t i2c_reg8_read16(i2c bus, uint8_t i2caddr, uint8_t reg)
{
	uint16_t data;
	uint8_t databuf[1];
	uint8_t addrbuf[2];
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data msgset;

	msgs[0].addr = i2caddr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = addrbuf;

	msgs[1].addr = i2caddr;
	msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
	msgs[1].len = 2;
	msgs[1].buf = databuf;

	msgset.msgs = msgs;
	msgset.nmsgs = 2;

	addrbuf[0] = reg;
	
	LOCK_MUTEX;

	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in i2c_read");
	}
	UNLOCK_MUTEX;

	data = databuf[0];
	data <<= 8;
	data |= databuf[1];

	return data;
}

// write 1 byte
void i2c_reg8_write8(i2c bus, uint8_t i2caddr, uint8_t reg, uint8_t data)
{
	uint8_t buf[2];
	struct i2c_msg msgs[1];
	struct i2c_rdwr_ioctl_data msgset;
	msgs[0].addr = i2caddr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgset.msgs = msgs;
	msgset.nmsgs = 1;

	buf[0] = reg;
	buf[1] = data;

	LOCK_MUTEX;
	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in write8");
	}
	UNLOCK_MUTEX;
}

// write 2 bytes
void i2c_reg8_write16(i2c bus, uint8_t i2caddr, uint8_t reg, uint16_t data)
{
	uint8_t buf[3];
	struct i2c_msg msgs[1];
	struct i2c_rdwr_ioctl_data msgset;
	msgs[0].addr = i2caddr;
	msgs[0].flags = 0;
	msgs[0].len = 3;
	msgs[0].buf = buf;

	msgset.msgs = msgs;
	msgset.nmsgs = 1;

	buf[0] = reg;
	buf[1] = data >> 8;
	buf[2] = data & 0xFF;

	LOCK_MUTEX;
	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in write8");
	}
	UNLOCK_MUTEX;
}

uint8_t i2c_reg16_read8(i2c bus, uint8_t i2caddr, uint16_t reg)
{
	uint8_t databuf[1];
	uint8_t addrbuf[2];
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data msgset;

	msgs[0].addr = i2caddr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = addrbuf;

	msgs[1].addr = i2caddr;
	msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
	msgs[1].len = 1;
	msgs[1].buf = databuf;

	msgset.msgs = msgs;
	msgset.nmsgs = 2;

	addrbuf[0] = reg >> 8;
	addrbuf[1] = reg & 0xFF;

	LOCK_MUTEX;
	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in i2c_read");
	}
	UNLOCK_MUTEX;
	
	return databuf[0];
}

uint16_t i2c_reg16_read16(i2c bus, uint8_t i2caddr, uint16_t reg)
{
	uint16_t data;
	uint8_t databuf[2];
	uint8_t addrbuf[2];
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data msgset;

	msgs[0].addr = i2caddr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (uint8_t *)addrbuf;

	msgs[1].addr = i2caddr;
	msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
	msgs[1].len = 2;
	msgs[1].buf = databuf;

	msgset.msgs = msgs;
	msgset.nmsgs = 2;

	addrbuf[0] = reg >> 8;
	addrbuf[1] = reg & 0xFF;

	LOCK_MUTEX;
	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in i2c_read");
	}
	UNLOCK_MUTEX;

	data = databuf[0];
	data <<= 8;
	data |= databuf[1];

	return data;
}

// write 1 byte
void i2c_reg16_write8(i2c bus, uint8_t i2caddr, uint16_t reg, uint8_t data)
{
	uint8_t buf[3];
	struct i2c_msg msgs[1];
	struct i2c_rdwr_ioctl_data msgset;
	msgs[0].addr = i2caddr;
	msgs[0].flags = 0;
	msgs[0].len = 3;
	msgs[0].buf = buf;

	msgset.msgs = msgs;
	msgset.nmsgs = 1;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = data;

	LOCK_MUTEX;
	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in write8");
	}
	UNLOCK_MUTEX;
}

// write 2 bytes
void i2c_reg16_write16(i2c bus, uint8_t i2caddr, uint16_t reg, uint16_t data)
{
	uint8_t buf[4];
	struct i2c_msg msgs[1];
	struct i2c_rdwr_ioctl_data msgset;
	msgs[0].addr = i2caddr;
	msgs[0].flags = 0;
	msgs[0].len = 4;
	msgs[0].buf = buf;

	msgset.msgs = msgs;
	msgset.nmsgs = 1;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = data >> 8;
	buf[3] = data & 0xFF;

	LOCK_MUTEX;
	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in write8");
	}
	UNLOCK_MUTEX;
}

void i2c_reg8_write16_n(i2c bus, uint8_t addr, uint8_t n, uint8_t *reg, uint16_t *data)
{
	uint8_t buf[3*n];
	struct i2c_msg msgs[n];
	struct i2c_rdwr_ioctl_data msgset;
	int i;

	for(i=0;i<n;i++) {
		msgs[0].addr = addr;
		msgs[0].flags = 0;
		msgs[0].len = 3;
		msgs[0].buf = buf + 3*i;
		buf[3*i] = reg[i];
		buf[3*i+1] = data[i] >> 8;
		buf[3*i+2] = data[i] & 0xFF;
	}
	msgset.msgs = msgs;
	msgset.nmsgs = n;
	LOCK_MUTEX;
	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in write8");
	}
	UNLOCK_MUTEX;
}

void i2c_reg8_write32_n(i2c bus, uint8_t addr, uint8_t n, uint8_t *reg, uint32_t *data)
{
	uint8_t buf[5 * n];
	struct i2c_msg msgs[n];
	struct i2c_rdwr_ioctl_data msgset;
	int i;

	for (i = 0; i < n; i++)
	{
		msgs[0].addr = addr;
		msgs[0].flags = 0;
		msgs[0].len = 3;
		msgs[0].buf = buf + 5 * i;
		buf[5 * i] = reg[i];
		buf[5 * i + 1] = (data[i] >> 24) & 0xFF;
		buf[5 * i + 2] = (data[i] >> 16) & 0xFF;
		buf[5 * i + 3] = (data[i] >>  8) & 0xFF;
		buf[5 * i + 4] = data[i] & 0xFF;
	}
	msgset.msgs = msgs;
	msgset.nmsgs = n;
	LOCK_MUTEX;
	if (ioctl(bus->fd, I2C_RDWR, &msgset) < 0)
	{
		DEBUG_INFO("ioctl(I2C_RDWR) in write8");
	}
	UNLOCK_MUTEX;
}

int i2c_read(i2c bus, int addr, unsigned char *buffer, int buffersize) {
	/* Lock mutex */
	LOCK_MUTEX;

	/* Join I2C. */
	if(ioctl(bus->fd, I2C_SLAVE, addr) < 0)
		return -1;

	/* Read from device */
	if(read(bus->fd, buffer, buffersize) != buffersize)
		return -1;

	/* Unlock mutex */
	UNLOCK_MUTEX;

	return 0;
}

void i2c_close(i2c bus) {
#ifdef USE_LOCKING
	pthread_mutex_destroy(&bus->mutex);
#endif
	close(bus->fd);
}
