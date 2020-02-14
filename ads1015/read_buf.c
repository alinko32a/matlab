extern int I2CFile;

double read_buf(uint8_t config_register)
{
    uint8_t writeBuf[3];	// Buffer to store the 3 bytes that we write to the I2C device
    uint8_t readBuf[2];	// 2 byte buffer to store the data read from the I2C device  
    int16_t val;		// Stores the 16 bit value of our ADC conversion

    // These three bytes are written to the ADS1115 to set the config register and start a conversion 
    writeBuf[0] = 1;			// This sets the pointer register so that the following two bytes write to the config register
    writeBuf[1] = config_register;   	// This sets the 8 MSBs of the config register (bits 15-8) to 11000011
    writeBuf[2] = 0x03;  		// This sets the 8 LSBs of the config register (bits 7-0) to 00000011

    // Initialize the buffer used to read data from the ADS1115 to 0
    readBuf[0]= 0;		
    readBuf[1]= 0;

    // Write writeBuf to the ADS1115, the 3 specifies the number of bytes we are writing,
    // this begins a single conversion
    write(I2CFile, writeBuf, 3);	

    // Wait for the conversion to complete, this requires bit 15 to change from 0->1
    while ((readBuf[0] & 0x80) == 0)	// readBuf[0] contains 8 MSBs of config register, AND with 10000000 to select bit 15
    {
      read(I2CFile, readBuf, 2);	// Read the config register into readBuf
    }

    writeBuf[0] = 0;					// Set pointer register to 0 to read from the conversion register
    write(I2CFile, writeBuf, 1);

    read(I2CFile, readBuf, 2);		// Read the contents of the conversion register into readBuf

    val = readBuf[0] << 8 | readBuf[1];	// Combine the two bytes of readBuf into a single 16 bit result 

    return val*4.096/32767.0;
}