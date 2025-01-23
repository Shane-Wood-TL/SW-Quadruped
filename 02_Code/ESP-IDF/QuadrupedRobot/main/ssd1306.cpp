#include "../include/drivers/ssd1306.h"
#include "../include/hardware_setup/pinout.h"

//display inputs

esp_err_t ssd1306::sendCommandSSD1306(uint8_t command)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // this is the current i2c "event" that is occuring
    // cmd stores all commands before executing them

    i2c_master_start(cmd); // start the event
    // set up writing the the correct address (master is writing to the address of the 1306)
    // i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_EN);
    i2c_master_write_byte(cmd, (ssd1306_address << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, 0x00, true);    // if data == 0, then command is being sent
    i2c_master_write_byte(cmd, command, true); // set the command that was given in parameter

    i2c_master_stop(cmd); // stop the transaction

    // this line actually does all of the lines above with a timeout period
    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_number, cmd, 1000 / portTICK_PERIOD_MS);

    i2c_cmd_link_delete(cmd); // delete the link/event
    return ret;               // return if the writing was sucessful
}

esp_err_t ssd1306::sendDataSSD1306(uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // this is the current i2c "event" that is occuring
    // cmd stores all commands before executing them
    i2c_master_start(cmd); // start the event

    // set up writing to the display
    i2c_master_write_byte(cmd, (ssd1306_address << 1) | I2C_MASTER_WRITE, true);

    // seting up data more (writting 0100_0000 to set D/C# to 1 (Data mode) in the control byte
    i2c_master_write_byte(cmd, 0x40, true);

    // write the data that was given to this function
    i2c_master_write(cmd, data, len, true);

    // stop the event
    i2c_master_stop(cmd);

    // this line actually does all of the lines above with a timeout period to the bus
    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_number, cmd, 1000 / portTICK_PERIOD_MS);
    // delete the link/event
    i2c_cmd_link_delete(cmd);
    return ret; // return if the writing was sucessful
}

void ssd1306::initSSD1306()
{
    // command list taken from mircocontrollers lab
    sendCommandSSD1306(0xAE);
    sendCommandSSD1306(0xA8); // Set MUX ratio
    sendCommandSSD1306(0x1F); // Set MUX ratio, 0x1F for 128x32 and 0x3F for 128x64
    sendCommandSSD1306(0xD3); // Set Display Offset
    sendCommandSSD1306(0x00); // Set Display Offset
    sendCommandSSD1306(0x40); // Set Display Start Line
    sendCommandSSD1306(0x20); // Set Display Mode to Horizontal
    sendCommandSSD1306(0x00); // Set Display Mode to Horizontal
    sendCommandSSD1306(0xA1); // Set Segment Re-map
    sendCommandSSD1306(0xC8); // Set COM Output Scan Direction
    sendCommandSSD1306(0xDA); // Set COM Pins hardware configuration
    sendCommandSSD1306(0x02); // Set COM Pins hardware configuration, 0x02 for 128x32 and 0x12 for 128x64
    sendCommandSSD1306(0x81); // Set Contrast Control
    sendCommandSSD1306(0x9F); // Set Contrast Control Value from 0x00 to 0xFF minimum to maximum
    sendCommandSSD1306(0xA4); // Disable Entire Display
    sendCommandSSD1306(0xA6); // Set Normal Display
    sendCommandSSD1306(0xD5); // Set Oscillation Frequency
    sendCommandSSD1306(0x80); // Set Oscillation Frequency
    sendCommandSSD1306(0x8D); // Enable Charge Pump Regulator
    sendCommandSSD1306(0x14); // Enable Charge Pump Regulator
    sendCommandSSD1306(0xAF); // Turn Display On
}

void ssd1306::clearSSD1306()
{
    uint8_t emptyBuffer[ssd1306_horizontal_resolution] = {0}; // each page is 128 x 8
    // display is 128 x 64, each page is 8 pixels high
    for (uint8_t i = 0; i < ssd1306_vertical_page_count; i++)
    {
        sendCommandSSD1306(0xB0 + i); // 0B0 - 0B7 are the 8 vertical pages

        // sets the column address from 0-127
        sendCommandSSD1306(0x00);
        sendCommandSSD1306(0x10);

        // write the empty buffer to the display
        sendDataSSD1306(emptyBuffer, sizeof(emptyBuffer));
    }
}

void ssd1306::drawPixelSSD1306(uint8_t x, uint8_t y)
{
    if (x >= ssd1306_horizontal_resolution || y >= ssd1306_vertical_resolution){
        return; // Out of bounds
    }
    uint8_t pixelInPage = (1 << (y % 8)); //limit the pixel from 0 to 7 with a 1 in the position 0000_0001, 0000_0010, etc
    //buffer is indexed as such
	// x straight across 0-127
	//each one of these is 8 pixels bits tall, or 1 page
	//repeat for pages 0-8
	//leading to a total size of 128*8 = 1024 page "strips" each containing the data for 8 pixels
	uint16_t bufferIndex = uint16_t(x + ((y / 8) * ssd1306_horizontal_resolution));
    buffer[bufferIndex] |= pixelInPage;  //write to the buffer based on page and column
}

void ssd1306::writeBufferSSD1306()
{
    for (uint8_t i = 0; i < 4; i++)
    {                                 // Send each page
        sendCommandSSD1306(0xB0 + i); // Set page start address
        sendCommandSSD1306(0x00);     // Set lower column address
        sendCommandSSD1306(0x10);     // Set higher column address

        sendDataSSD1306(&buffer[128 * i], 128);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void ssd1306::writeLetterSSD1306(uint8_t letter, uint8_t xPos, uint8_t yPos)
{

    const uint16_t *matrix = fontInstance.getLetter(letter);//get the pixel data for one font char
    for (int i = 0; i < 16; i++)
    { // 16 tall
        for (int j = 0; j < 12; j++)
        { // 12 wide
            if ((matrix[i]) & (1 << (12 - j)))
            {
                drawPixelSSD1306(xPos + j, yPos + i); //draw the pixel to buffer if on
            }
        }
    }
}

void ssd1306::writeStringSSD1306(std::string Word, uint8_t xPos, uint8_t yPos)
{
    for (uint8_t i = 0; i < Word.length(); i++)
    {
        writeLetterSSD1306(Word[i], xPos + i * 12, yPos); //each char is 12 wide, go through each letter
        //with proper spacing
    }
}

ssd1306::ssd1306()
{
    initSSD1306(); // set up display
    clearSSD1306(); //clear display

    // Send the empty buffer to the display
    writeBufferSSD1306();
}

// empties buffer so that new data can be put into it
void ssd1306::clearBuffer()
{
    //reset the buffer to all 0's
    for (uint16_t i = 0; i < 512; i++)
    {
        buffer[i] = 0;
    }
    return;
}


void ssd1306::update(){
    clearBuffer();
}


void ssd1306::writeSection(const uint8_t page,const uint8_t columnStart, const uint8_t columnStop){
	sendCommandSSD1306(0x22);
	sendCommandSSD1306(page);
	sendCommandSSD1306(page);
	sendCommandSSD1306(0x21);
	sendCommandSSD1306(columnStart);
	sendCommandSSD1306(columnStop);
	return;
}