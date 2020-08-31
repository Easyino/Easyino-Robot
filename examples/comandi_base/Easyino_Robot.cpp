
/*
      //////////////////|||\\\\\\\\\\\\\\\\\\\\
     ||| Easyino_Robot - Education Official Sketch |||
      \\\\\\\\\\\\\\\\\\|||////////////////////
  Easyino_Robot.cpp - Library for using our inboard components in the best way
  Created by Federico Longhin, Mattia Pilotto, Edoardo Baggio 03 April, 2020.
  Manteined by Federico Longhin & Mattia Pilotto.
  V 0.3f
*/
//generale

#include "Easyino_Robot.h"
#include "Arduino.h"
#include <EEPROM.h>
#include <SPI.h>
#define mode 1



const byte anim[5][15] {
  {0x04, 0x00, 0x06, 0x80, 0x07, 0x80, 0xFF},
  {0x20, 0x00, 0x60, 0x01, 0xE0, 0x01, 0xFF},
  {0x18, 0x00, 0x18, 0x00, 0xFF},
  {0x00, 0x3C, 0x00, 0x7E, 0xFF},
  {0x10, 0xC3, 0x08, 0x66, 0x10, 0x3C, 0x08, 0x18, 0x10, 0x66, 0x08, 0xC3, 0x10, 0xC3, 0xFF}
};
const int duranim[5] {600, 600, 700, 700, 250};
byte r1 = 0x00;
byte r2 = 0x00;
int r, c, f;
int v;
unsigned int g = 0;
int an = -1;
const int ncom = 3;
const int clockpin = 8;
const int latchpin = 7;
const int datapin = 4;
unsigned long int kcm = 0;
unsigned long int kgr = 0;
int tempo_inizio;
int tempo_tarato;
void registri() {
  digitalWrite(latchpin, LOW);
  shiftOut(datapin, clockpin, MSBFIRST, r1);
  shiftOut(datapin, clockpin, MSBFIRST, r2);
  digitalWrite(latchpin, HIGH);
}
void spegni_led() {
  r1 = 0;
  r2 = 0;
  registri();
  v = 0;
  an = -1;
}
void ferma_motori() {
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  analogWrite(3, 0);
  analogWrite(6, 0);
}
int lunghezza(int com) {                            // Calcola la quantità di cambiamenti nell'animazione
  for (c = 0; anim[com][c] != 0xFF; c++) {}
  return c / 2;
}


//--------------------------------------------------------------------------------------------------------------------------------


// End constructor

/**
   Constructor.
   Prepares the output pins.
*/
int robot_name;
bool isEasy;
Easyino_Robot::Easyino_Robot(int r_n)
{
  robot_name = r_n;
  isEasy = r_n;
  _chipSelectPin = 10;
  _resetPowerDownPin = 9;
}
//Easyino_Robot::Easyino_Robot(byte chipSelectPin,  ///< Arduino pin connected to Easyino_Robot's SPI slave select input (Pin 24, NSS, active low)
//                             byte resetPowerDownPin ///< Arduino pin connected to Easyino_Robot's reset and power down input (Pin 6, NRSTPD, active low)
//                            )
//{
//  _chipSelectPin = chipSelectPin;
//  _resetPowerDownPin = resetPowerDownPin;
//} // End constructor

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the Easyino_Robot
/////////////////////////////////////////////////////////////////////////////////////

/**
   Writes a byte to the specified register in the Easyino_Robot chip.
   The interface is described in the datasheet section 8.1.2.
*/
void Easyino_Robot::PCD_WriteRegister(byte reg,  ///< The register to write to. One of the PCD_Register enums.
                                      byte value ///< The value to write.
                                     )
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0)); // Set the settings to work with SPI bus
  digitalWrite(_chipSelectPin, LOW);                    // Select slave
  SPI.transfer(reg & 0x7E);                       // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
  SPI.transfer(value);
  digitalWrite(_chipSelectPin, HIGH); // Release slave again
  SPI.endTransaction();       // Stop using the SPI bus
} // End PCD_WriteRegister()

/**
   Writes a number of bytes to the specified register in the Easyino_Robot chip.
   The interface is described in the datasheet section 8.1.2.
*/
void Easyino_Robot::PCD_WriteRegister(byte reg,  ///< The register to write to. One of the PCD_Register enums.
                                      byte count,  ///< The number of bytes to write to the register
                                      byte *values ///< The values to write. Byte array.
                                     )
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0)); // Set the settings to work with SPI bus
  digitalWrite(_chipSelectPin, LOW);                    // Select slave
  SPI.transfer(reg & 0x7E);                       // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
  for (byte index = 0; index < count; index++)
  {
    SPI.transfer(values[index]);
  }
  digitalWrite(_chipSelectPin, HIGH); // Release slave again
  SPI.endTransaction();       // Stop using the SPI bus
} // End PCD_WriteRegister()

/**
   Reads a byte from the specified register in the Easyino_Robot chip.
   The interface is described in the datasheet section 8.1.2.
*/
byte Easyino_Robot::PCD_ReadRegister(byte reg ///< The register to read from. One of the PCD_Register enums.
                                    )
{
  byte value;
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0)); // Set the settings to work with SPI bus
  digitalWrite(_chipSelectPin, LOW);                    // Select slave
  SPI.transfer(0x80 | (reg & 0x7E));                    // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  value = SPI.transfer(0);                        // Read the value back. Send 0 to stop reading.
  digitalWrite(_chipSelectPin, HIGH);                   // Release slave again
  SPI.endTransaction();                         // Stop using the SPI bus
  return value;
} // End PCD_ReadRegister()

/**
   Reads a number of bytes from the specified register in the Easyino_Robot chip.
   The interface is described in the datasheet section 8.1.2.
*/
void Easyino_Robot::PCD_ReadRegister(byte reg,   ///< The register to read from. One of the PCD_Register enums.
                                     byte count,   ///< The number of bytes to read
                                     byte *values, ///< Byte array to store the values in.
                                     byte rxAlign  ///< Only bit positions rxAlign..7 in values[0] are updated.
                                    )
{
  if (count == 0)
  {
    return;
  }
  //Serial.print(F("Reading "));  Serial.print(count); Serial.println(F(" bytes from register."));
  byte address = 0x80 | (reg & 0x7E);                   // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  byte index = 0;                             // Index in values array.
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0)); // Set the settings to work with SPI bus
  digitalWrite(_chipSelectPin, LOW);                    // Select slave
  count--;                                // One read is performed outside of the loop
  SPI.transfer(address);                          // Tell Easyino_Robot which address we want to read
  while (index < count)
  {
    if (index == 0 && rxAlign)
    { // Only update bit positions rxAlign..7 in values[0]
      // Create bit mask for bit positions rxAlign..7
      byte mask = 0;
      for (byte i = rxAlign; i <= 7; i++)
      {
        mask |= (1 << i);
      }
      // Read value and tell that we want to read the same address again.
      byte value = SPI.transfer(address);
      // Apply mask to both current value of values[0] and the new data in value.
      values[0] = (values[index] & ~mask) | (value & mask);
    }
    else
    { // Normal case
      values[index] = SPI.transfer(address); // Read value and tell that we want to read the same address again.
    }
    index++;
  }
  values[index] = SPI.transfer(0);  // Read the final byte. Send 0 to stop reading.
  digitalWrite(_chipSelectPin, HIGH); // Release slave again
  SPI.endTransaction();       // Stop using the SPI bus
} // End PCD_ReadRegister()

/**
   Sets the bits given in mask in register reg.
*/
void Easyino_Robot::PCD_SetRegisterBitMask(byte reg, ///< The register to update. One of the PCD_Register enums.
    byte mask ///< The bits to set.
                                          )
{
  byte tmp;
  tmp = PCD_ReadRegister(reg);
  PCD_WriteRegister(reg, tmp | mask); // set bit mask
} // End PCD_SetRegisterBitMask()

/**
   Clears the bits given in mask from register reg.
*/
void Easyino_Robot::PCD_ClearRegisterBitMask(byte reg, ///< The register to update. One of the PCD_Register enums.
    byte mask ///< The bits to clear.
                                            )
{
  byte tmp;
  tmp = PCD_ReadRegister(reg);
  PCD_WriteRegister(reg, tmp & (~mask)); // clear bit mask
} // End PCD_ClearRegisterBitMask()

/**
   Use the CRC coprocessor in the Easyino_Robot to calculate a CRC_A.

   @return STATUS_OK on success, STATUS_??? otherwise.
*/
Easyino_Robot::StatusCode Easyino_Robot::PCD_CalculateCRC(byte *data,  ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
    byte length, ///< In: The number of bytes to transfer.
    byte *result ///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
                                                         )
{
  PCD_WriteRegister(CommandReg, PCD_Idle);    // Stop any active command.
  PCD_WriteRegister(DivIrqReg, 0x04);       // Clear the CRCIRq interrupt request bit
  PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);   // FlushBuffer = 1, FIFO initialization
  PCD_WriteRegister(FIFODataReg, length, data); // Write data to the FIFO
  PCD_WriteRegister(CommandReg, PCD_CalcCRC);   // Start the calculation

  // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73�s.
  word i = 5000;
  byte n;
  while (1)
  {
    n = PCD_ReadRegister(DivIrqReg); // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
    if (n & 0x04)
    { // CRCIRq bit set - calculation done
      break;
    }
    if (--i == 0)
    { // The emergency break. We will eventually terminate on this one after 89ms. Communication with the Easyino_Robot might be down.
      return STATUS_TIMEOUT;
    }
  }
  PCD_WriteRegister(CommandReg, PCD_Idle); // Stop calculating CRC for new content in the FIFO.

  // Transfer the result from the registers to the result buffer
  result[0] = PCD_ReadRegister(CRCResultRegL);
  result[1] = PCD_ReadRegister(CRCResultRegH);
  return STATUS_OK;
} // End PCD_CalculateCRC()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the Easyino_Robot
/////////////////////////////////////////////////////////////////////////////////////

/**
   Initializes the Easyino_Robot chip.
*/
void Easyino_Robot::PCD_Init()
{
  // Set the chipSelectPin as digital output, do not select the slave yet
  pinMode(_chipSelectPin, OUTPUT);
  digitalWrite(_chipSelectPin, HIGH);

  // Set the resetPowerDownPin as digital output, do not reset or power down.
  pinMode(_resetPowerDownPin, OUTPUT);

  if (digitalRead(_resetPowerDownPin) == LOW)
  { //The Easyino_Robot chip is in power down mode.
    digitalWrite(_resetPowerDownPin, HIGH); // Exit power down mode. This triggers a hard reset.
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
    delay(50);
  }
  else
  { // Perform a soft reset
    PCD_Reset();
  }

  // When communicating with a PICC we need a timeout if something goes wrong.
  // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
  // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
  PCD_WriteRegister(TModeReg, 0x80);    // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  PCD_WriteRegister(TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
  PCD_WriteRegister(TReloadRegH, 0x03); // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
  PCD_WriteRegister(TReloadRegL, 0xE8);

  PCD_WriteRegister(TxASKReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
  PCD_WriteRegister(ModeReg, 0x3D);  // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  PCD_AntennaOn();           // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
   Initializes the Easyino_Robot chip.
*/
void Easyino_Robot::PCD_Init(byte chipSelectPin,    ///< Arduino pin connected to Easyino_Robot's SPI slave select input (Pin 24, NSS, active low)
                             byte resetPowerDownPin ///< Arduino pin connected to Easyino_Robot's reset and power down input (Pin 6, NRSTPD, active low)
                            )
{
  _chipSelectPin = chipSelectPin;
  _resetPowerDownPin = resetPowerDownPin;
  // Set the chipSelectPin as digital output, do not select the slave yet
  PCD_Init();
} // End PCD_Init()

/**
   Performs a soft reset on the Easyino_Robot chip and waits for it to be ready again.
*/
void Easyino_Robot::PCD_Reset()
{
  PCD_WriteRegister(CommandReg, PCD_SoftReset); // Issue the SoftReset command.
  // The datasheet does not mention how long the SoftRest command takes to complete.
  // But the Easyino_Robot might have been in soft power-down mode (triggered by bit 4 of CommandReg)
  // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
  delay(50);
  // Wait for the PowerDown bit in CommandReg to be cleared
  while (PCD_ReadRegister(CommandReg) & (1 << 4))
  {
    // PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
  }
} // End PCD_Reset()

/**
   Turns the antenna on by enabling pins TX1 and TX2.
   After a reset these pins are disabled.
*/
void Easyino_Robot::PCD_AntennaOn()
{
  byte value = PCD_ReadRegister(TxControlReg);
  if ((value & 0x03) != 0x03)
  {
    PCD_WriteRegister(TxControlReg, value | 0x03);
  }
} // End PCD_AntennaOn()

/**
   Turns the antenna off by disabling pins TX1 and TX2.
*/
void Easyino_Robot::PCD_AntennaOff()
{
  PCD_ClearRegisterBitMask(TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
   Get the current Easyino_Robot Receiver Gain (RxGain[2:0]) value.
   See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/Easyino_Robot.pdf
   NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.

   @return Value of the RxGain, scrubbed to the 3 bits used.
*/
byte Easyino_Robot::PCD_GetAntennaGain()
{
  return PCD_ReadRegister(RFCfgReg) & (0x07 << 4);
} // End PCD_GetAntennaGain()

/**
   Set the Easyino_Robot Receiver Gain (RxGain) to value specified by given mask.
   See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/Easyino_Robot.pdf
   NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
*/
void Easyino_Robot::PCD_SetAntennaGain(byte mask)
{
  if (PCD_GetAntennaGain() != mask)
  { // only bother if there is a change
    PCD_ClearRegisterBitMask(RFCfgReg, (0x07 << 4));    // clear needed to allow 000 pattern
    PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07 << 4)); // only set RxGain[2:0] bits
  }
} // End PCD_SetAntennaGain()

/**
   Performs a self-test of the Easyino_Robot
   See 16.1.1 in http://www.nxp.com/documents/data_sheet/Easyino_Robot.pdf

   @return Whether or not the test passed. Or false if no firmware reference is available.
*/
bool Easyino_Robot::PCD_PerformSelfTest()
{
  // This follows directly the steps outlined in 16.1.1
  // 1. Perform a soft reset.
  PCD_Reset();

  // 2. Clear the internal buffer by writing 25 bytes of 00h
  byte ZEROES[25] = {0x00};
  PCD_SetRegisterBitMask(FIFOLevelReg, 0x80); // flush the FIFO buffer
  PCD_WriteRegister(FIFODataReg, 25, ZEROES); // write 25 bytes of 00h to FIFO
  PCD_WriteRegister(CommandReg, PCD_Mem);   // transfer to internal buffer

  // 3. Enable self-test
  PCD_WriteRegister(AutoTestReg, 0x09);

  // 4. Write 00h to FIFO buffer
  PCD_WriteRegister(FIFODataReg, 0x00);

  // 5. Start self-test by issuing the CalcCRC command
  PCD_WriteRegister(CommandReg, PCD_CalcCRC);

  // 6. Wait for self-test to complete
  word i;
  byte n;
  for (i = 0; i < 0xFF; i++)
  {
    n = PCD_ReadRegister(DivIrqReg); // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
    if (n & 0x04)
    { // CRCIRq bit set - calculation done
      break;
    }
  }
  PCD_WriteRegister(CommandReg, PCD_Idle); // Stop calculating CRC for new content in the FIFO.

  // 7. Read out resulting 64 bytes from the FIFO buffer.
  byte result[64];
  PCD_ReadRegister(FIFODataReg, 64, result, 0);

  // Auto self-test done
  // Reset AutoTestReg register to be 0 again. Required for normal operation.
  PCD_WriteRegister(AutoTestReg, 0x00);

  // Determine firmware version (see section 9.3.4.8 in spec)
  byte version = PCD_ReadRegister(VersionReg);

  // Pick the appropriate reference values
  const byte *reference;
  switch (version)
  {
    case 0x88: // Fudan Semiconductor FM17522 clone
      reference = FM17522_firmware_reference;
      break;
    case 0x90: // Version 0.0
      reference = Easyino_Robot_firmware_referenceV0_0;
      break;
    case 0x91: // Version 1.0
      reference = Easyino_Robot_firmware_referenceV1_0;
      break;
    case 0x92: // Version 2.0
      reference = Easyino_Robot_firmware_referenceV2_0;
      break;
    default:      // Unknown version
      return false; // abort test
  }

  // Verify that the results match up to our expectations
  for (i = 0; i < 64; i++)
  {
    if (result[i] != pgm_read_byte(&(reference[i])))
    {
      return false;
    }
  }

  // Test passed; all is good.
  return true;
} // End PCD_PerformSelfTest()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
   Executes the Transceive command.
   CRC validation can only be done if backData and backLen are specified.

   @return STATUS_OK on success, STATUS_??? otherwise.
*/
Easyino_Robot::StatusCode Easyino_Robot::PCD_TransceiveData(byte *sendData,  ///< Pointer to the data to transfer to the FIFO.
    byte sendLen,  ///< Number of bytes to transfer to the FIFO.
    byte *backData,  ///< NULL or pointer to buffer if data should be read back after executing the command.
    byte *backLen,   ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
    byte *validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
    byte rxAlign,  ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
    bool checkCRC  ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
                                                           )
{
  byte waitIRq = 0x30; // RxIRq and IdleIRq
  return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
   Transfers data to the Easyino_Robot FIFO, executes a command, waits for completion and transfers data back from the FIFO.
   CRC validation can only be done if backData and backLen are specified.

   @return STATUS_OK on success, STATUS_??? otherwise.
*/
Easyino_Robot::StatusCode Easyino_Robot::PCD_CommunicateWithPICC(byte command,    ///< The command to execute. One of the PCD_Command enums.
    byte waitIRq,    ///< The bits in the ComIrqReg register that signals successful completion of the command.
    byte *sendData,  ///< Pointer to the data to transfer to the FIFO.
    byte sendLen,    ///< Number of bytes to transfer to the FIFO.
    byte *backData,  ///< NULL or pointer to buffer if data should be read back after executing the command.
    byte *backLen,   ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
    byte *validBits, ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
    byte rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
    bool checkCRC    ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
                                                                )
{
  byte n, _validBits;
  unsigned int i;

  // Prepare values for BitFramingReg
  byte txLastBits = validBits ? *validBits : 0;
  byte bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

  PCD_WriteRegister(CommandReg, PCD_Idle);       // Stop any active command.
  PCD_WriteRegister(ComIrqReg, 0x7F);          // Clear all seven interrupt request bits
  PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);      // FlushBuffer = 1, FIFO initialization
  PCD_WriteRegister(FIFODataReg, sendLen, sendData); // Write sendData to the FIFO
  PCD_WriteRegister(BitFramingReg, bitFraming);    // Bit adjustments
  PCD_WriteRegister(CommandReg, command);        // Execute the command
  if (command == PCD_Transceive)
  {
    PCD_SetRegisterBitMask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
  }

  // Wait for the command to complete.
  // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
  // Each iteration of the do-while-loop takes 17.86�s.
  i = 2000;
  while (1)
  {
    n = PCD_ReadRegister(ComIrqReg); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
    if (n & waitIRq)
    { // One of the interrupts that signal success has been set.
      break;
    }
    if (n & 0x01)
    { // Timer interrupt - nothing received in 25ms
      return STATUS_TIMEOUT;
    }
    if (--i == 0)
    { // The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the Easyino_Robot might be down.
      return STATUS_TIMEOUT;
    }
  }

  // Stop now if any errors except collisions were detected.
  byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
  if (errorRegValue & 0x13)
  { // BufferOvfl ParityErr ProtocolErr
    return STATUS_ERROR;
  }

  // If the caller wants data back, get it from the Easyino_Robot.
  if (backData && backLen)
  {
    n = PCD_ReadRegister(FIFOLevelReg); // Number of bytes in the FIFO
    if (n > *backLen)
    {
      return STATUS_NO_ROOM;
    }
    *backLen = n;                    // Number of bytes returned
    PCD_ReadRegister(FIFODataReg, n, backData, rxAlign); // Get received data from FIFO
    _validBits = PCD_ReadRegister(ControlReg) & 0x07;  // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
    if (validBits)
    {
      *validBits = _validBits;
    }
  }

  // Tell about collisions
  if (errorRegValue & 0x08)
  { // CollErr
    return STATUS_COLLISION;
  }

  // Perform CRC_A validation if requested.
  if (backData && backLen && checkCRC)
  {
    // In this case a MIFARE Classic NAK is not OK.
    if (*backLen == 1 && _validBits == 4)
    {
      return STATUS_MIFARE_NACK;
    }
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if (*backLen < 2 || _validBits != 0)
    {
      return STATUS_CRC_WRONG;
    }
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    byte controlBuffer[2];
    Easyino_Robot::StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
    if (status != STATUS_OK)
    {
      return status;
    }
    if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
    {
      return STATUS_CRC_WRONG;
    }
  }

  return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
   Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
   Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.

   @return STATUS_OK on success, STATUS_??? otherwise.
*/
Easyino_Robot::StatusCode Easyino_Robot::PICC_RequestA(byte *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
    byte *bufferSize  ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
                                                      )
{
  return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
   Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
   Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.

   @return STATUS_OK on success, STATUS_??? otherwise.
*/
Easyino_Robot::StatusCode Easyino_Robot::PICC_WakeupA(byte *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
    byte *bufferSize  ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
                                                     )
{
  return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
   Transmits REQA or WUPA commands.
   Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.

   @return STATUS_OK on success, STATUS_??? otherwise.
*/
Easyino_Robot::StatusCode Easyino_Robot::PICC_REQA_or_WUPA(byte command,   ///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
    byte *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
    byte *bufferSize  ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
                                                          )
{
  byte validBits;
  Easyino_Robot::StatusCode status;

  if (bufferATQA == NULL || *bufferSize < 2)
  { // The ATQA response is 2 bytes long.
    return STATUS_NO_ROOM;
  }
  PCD_ClearRegisterBitMask(CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.
  validBits = 7;               // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
  status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
  if (status != STATUS_OK)
  {
    return status;
  }
  if (*bufferSize != 2 || validBits != 0)
  { // ATQA must be exactly 16 bits.
    return STATUS_ERROR;
  }
  return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
   Transmits SELECT/ANTICOLLISION commands to select a single PICC.
   Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
   On success:
      - The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
      - The UID size and value of the chosen PICC is returned in *uid along with the SAK.

   A PICC UID consists of 4, 7 or 10 bytes.
   Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
      UID size  Number of UID bytes   Cascade levels    Example of PICC
      ========  ===================   ==============    ===============
      single         4            1       MIFARE Classic
      double         7            2       MIFARE Ultralight
      triple        10            3       Not currently in use?

   @return STATUS_OK on success, STATUS_??? otherwise.
*/
Easyino_Robot::StatusCode Easyino_Robot::PICC_Select(Uid *uid,    ///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
    byte validBits ///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
                                                    )
{
  bool uidComplete;
  bool selectDone;
  bool useCascadeTag;
  byte cascadeLevel = 1;
  Easyino_Robot::StatusCode result;
  byte count;
  byte index;
  byte uidIndex;          // The first index in uid->uidByte[] that is used in the current Cascade Level.
  int8_t currentLevelKnownBits; // The number of known UID bits in the current Cascade Level.
  byte buffer[9];         // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
  byte bufferUsed;        // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
  byte rxAlign;         // Used in BitFramingReg. Defines the bit position for the first bit received.
  byte txLastBits;        // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
  byte *responseBuffer;
  byte responseLength;

  // Description of buffer structure:
  //    Byte 0: SEL         Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
  //    Byte 1: NVB         Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
  //    Byte 2: UID-data or CT    See explanation below. CT means Cascade Tag.
  //    Byte 3: UID-data
  //    Byte 4: UID-data
  //    Byte 5: UID-data
  //    Byte 6: BCC         Block Check Character - XOR of bytes 2-5
  //    Byte 7: CRC_A
  //    Byte 8: CRC_A
  // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
  //
  // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
  //    UID size  Cascade level Byte2 Byte3 Byte4 Byte5
  //    ========  ============= ===== ===== ===== =====
  //     4 bytes    1     uid0  uid1  uid2  uid3
  //     7 bytes    1     CT    uid0  uid1  uid2
  //            2     uid3  uid4  uid5  uid6
  //    10 bytes    1     CT    uid0  uid1  uid2
  //            2     CT    uid3  uid4  uid5
  //            3     uid6  uid7  uid8  uid9

  // Sanity checks
  if (validBits > 80)
  {
    return STATUS_INVALID;
  }

  // Prepare Easyino_Robot
  PCD_ClearRegisterBitMask(CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.

  // Repeat Cascade Level loop until we have a complete UID.
  uidComplete = false;
  while (!uidComplete)
  {
    // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
    switch (cascadeLevel)
    {
      case 1:
        buffer[0] = PICC_CMD_SEL_CL1;
        uidIndex = 0;
        useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 bytes
        break;

      case 2:
        buffer[0] = PICC_CMD_SEL_CL2;
        uidIndex = 3;
        useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 bytes
        break;

      case 3:
        buffer[0] = PICC_CMD_SEL_CL3;
        uidIndex = 6;
        useCascadeTag = false; // Never used in CL3.
        break;

      default:
        return STATUS_INTERNAL_ERROR;
        break;
    }

    // How many UID bits are known in this Cascade Level?
    currentLevelKnownBits = validBits - (8 * uidIndex);
    if (currentLevelKnownBits < 0)
    {
      currentLevelKnownBits = 0;
    }
    // Copy the known bits from uid->uidByte[] to buffer[]
    index = 2; // destination index in buffer[]
    if (useCascadeTag)
    {
      buffer[index++] = PICC_CMD_CT;
    }
    byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
    if (bytesToCopy)
    {
      byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
      if (bytesToCopy > maxBytes)
      {
        bytesToCopy = maxBytes;
      }
      for (count = 0; count < bytesToCopy; count++)
      {
        buffer[index++] = uid->uidByte[uidIndex + count];
      }
    }
    // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
    if (useCascadeTag)
    {
      currentLevelKnownBits += 8;
    }

    // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
    selectDone = false;
    while (!selectDone)
    {
      // Find out how many bits and bytes to send and receive.
      if (currentLevelKnownBits >= 32)
      { // All UID bits in this Cascade Level are known. This is a SELECT.
        //Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
        buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
        // Calculate BCC - Block Check Character
        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
        // Calculate CRC_A
        result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
        if (result != STATUS_OK)
        {
          return result;
        }
        txLastBits = 0; // 0 => All 8 bits are valid.
        bufferUsed = 9;
        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
        responseBuffer = &buffer[6];
        responseLength = 3;
      }
      else
      { // This is an ANTICOLLISION.
        //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
        txLastBits = currentLevelKnownBits % 8;
        count = currentLevelKnownBits / 8;     // Number of whole bytes in the UID part.
        index = 2 + count;             // Number of whole bytes: SEL + NVB + UIDs
        buffer[1] = (index << 4) + txLastBits; // NVB - Number of Valid Bits
        bufferUsed = index + (txLastBits ? 1 : 0);
        // Store response in the unused part of buffer
        responseBuffer = &buffer[index];
        responseLength = sizeof(buffer) - index;
      }

      // Set bit adjustments
      rxAlign = txLastBits;                      // Having a separate variable is overkill. But it makes the next line easier to read.
      PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

      // Transmit the buffer and receive the response.
      result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
      if (result == STATUS_COLLISION)
      { // More than one PICC in the field => collision.
        byte valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
        if (valueOfCollReg & 0x20)
        { // CollPosNotValid
          return STATUS_COLLISION; // Without a valid collision position we cannot continue
        }
        byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
        if (collisionPos == 0)
        {
          collisionPos = 32;
        }
        if (collisionPos <= currentLevelKnownBits)
        { // No progress - should not happen
          return STATUS_INTERNAL_ERROR;
        }
        // Choose the PICC with the bit set.
        currentLevelKnownBits = collisionPos;
        count = (currentLevelKnownBits - 1) % 8;           // The bit to modify
        index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
        buffer[index] |= (1 << count);
      }
      else if (result != STATUS_OK)
      {
        return result;
      }
      else
      { // STATUS_OK
        if (currentLevelKnownBits >= 32)
        { // This was a SELECT.
          selectDone = true; // No more anticollision
          // We continue below outside the while.
        }
        else
        { // This was an ANTICOLLISION.
          // We now have all 32 bits of the UID in this Cascade Level
          currentLevelKnownBits = 32;
          // Run loop again to do the SELECT.
        }
      }
    } // End of while (!selectDone)

    // We do not check the CBB - it was constructed by us above.

    // Copy the found UID bytes from buffer[] to uid->uidByte[]
    index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
    bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
    for (count = 0; count < bytesToCopy; count++)
    {
      uid->uidByte[uidIndex + count] = buffer[index++];
    }

    // Check response SAK (Select Acknowledge)
    if (responseLength != 3 || txLastBits != 0)
    { // SAK must be exactly 24 bits (1 byte + CRC_A).
      return STATUS_ERROR;
    }
    // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
    result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
    if (result != STATUS_OK)
    {
      return result;
    }
    if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
    {
      return STATUS_CRC_WRONG;
    }
    if (responseBuffer[0] & 0x04)
    { // Cascade bit set - UID not complete yes
      cascadeLevel++;
    }
    else
    {
      uidComplete = true;
      uid->sak = responseBuffer[0];
    }
  } // End of while (!uidComplete)

  // Set correct uid->size
  uid->size = 3 * cascadeLevel + 1;

  return STATUS_OK;
} // End PICC_Select()

/**
   Instructs a PICC in state ACTIVE(*) to go to state HALT.

   @return STATUS_OK on success, STATUS_??? otherwise.
*/
Easyino_Robot::StatusCode Easyino_Robot::PICC_HaltA()
{
  Easyino_Robot::StatusCode result;
  byte buffer[4];

  // Build command buffer
  buffer[0] = PICC_CMD_HLTA;
  buffer[1] = 0;
  // Calculate CRC_A
  result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
  if (result != STATUS_OK)
  {
    return result;
  }

  // Send the command.
  // The standard says:
  //    If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
  //    HLTA command, this response shall be interpreted as 'not acknowledge'.
  // We interpret that this way: Only STATUS_TIMEOUT is a success.
  result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0);
  if (result == STATUS_TIMEOUT)
  {
    return STATUS_OK;
  }
  if (result == STATUS_OK)
  { // That is ironically NOT ok in this case ;-)
    return STATUS_ERROR;
  }
  return result;
} // End PICC_HaltA()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
   Executes the Easyino_Robot MFAuthent command.
   This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
   The authentication is described in the Easyino_Robot datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
   For use with MIFARE Classic PICCs.
   The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
   Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.

   All keys are set to FFFFFFFFFFFFh at chip delivery.

   @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
*/
Easyino_Robot::StatusCode Easyino_Robot::PCD_Authenticate(byte command,    ///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
    byte blockAddr,  ///< The block number. See numbering in the comments in the .h file.
    MIFARE_Key *key, ///< Pointer to the Crypteo1 key to use (6 bytes)
    Uid *uid       ///< Pointer to Uid struct. The first 4 bytes of the UID is used.
                                                         )
{
  byte waitIRq = 0x10; // IdleIRq

  // Build command buffer
  byte sendData[12];
  sendData[0] = command;
  sendData[1] = blockAddr;
  for (byte i = 0; i < MF_KEY_SIZE; i++)
  { // 6 key bytes
    sendData[2 + i] = key->keyByte[i];
  }
  for (byte i = 0; i < 4; i++)
  { // The first 4 bytes of the UID
    sendData[8 + i] = uid->uidByte[i];
  }

  // Start the authentication.
  return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData));
} // End PCD_Authenticate()

/**
   Used to exit the PCD from its authenticated state.
   Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
*/
void Easyino_Robot::PCD_StopCrypto1()
{
  // Clear MFCrypto1On bit
  PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
   Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.

   For MIFARE Classic the sector containing the block must be authenticated before calling this function.

   For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
   The MF0ICU1 returns a NAK for higher addresses.
   The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
   For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
   A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.

   The buffer must be at least 18 bytes because a CRC_A is also returned.
   Checks the CRC_A before returning STATUS_OK.

   @return STATUS_OK on success, STATUS_??? otherwise.
*/


/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
   Returns true if a PICC responds to PICC_CMD_REQA.
   Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.

   @return bool
*/
bool Easyino_Robot::PICC_IsNewCardPresent()
{
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  Easyino_Robot::StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
  return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
   Simple wrapper around PICC_Select.
   Returns true if a UID could be read.
   Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
   The read UID is available in the class variable uid.

   @return bool
*/
bool Easyino_Robot::PICC_ReadCardSerial()
{
  Easyino_Robot::StatusCode result = PICC_Select(&uid);
  return (result == STATUS_OK);
} // End




/////////////////////////////////////////////////////////////////////////////////////
// funzioni aggiunte da  noi
/////////////////////////////////////////////////////////////////////////////////////
void Easyino_Robot::begin() {
#define SS_PIN 10
#define RST_PIN 9



  MIFARE_Key key;


  byte nuidPICC[3];
  Serial.begin(9600);
  SPI.begin(); // Init SPI bus
  PCD_Init(); // Init MFRC522

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  // MOTORI
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  // VELOCITA' MOTORI
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  // REGISTRI
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  ferma_motori();

  kcm = EEPROM.read(0); //distanza in avanti
  kgr = EEPROM.read(1); //distanza in girare
  kdx = EEPROM.read(2); //coeficente giro per ruota destra
  ksx = EEPROM.read(3); //coeficente giro per ruota destra

  Serial.println(F("ciao, io sono pronto... e tu brutto strunz? \nappoggia una carta o esploderò tra 10 secondi."));
  digitalWrite(3, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(A4, LOW);
  animazioneTagRiconosciuto();
}

bool  Easyino_Robot::riceve_qualcosa() {
  if (PICC_IsNewCardPresent()) {
    if (PICC_ReadCardSerial()) {
      spegni_led();
      g = 1;
      return true;
    }
  }
  if (an == -1) {
    g = (g + 1) % 50;
    if (g == 0) {
      if (f == 1) {
        r1 = 0xE7;
        r2 = 0x81;
        registri();
        f = 0;
      }
      else {
        spegni_led();
        f = 1;
      }
    }
  }
  else {
    switch (an) {
      case 0: {
          r1 = 0x18;
          r2 = 0x00;
          registri();
          break;
        }
      case 1: {
          r1 = 0x00;
          r2 = 0x7E;
          registri();
          break;
        }
      case 2: {
          r1 = 0x07;
          r2 = 0x80;
          registri();
          break;
        }
      case 3: {
          r1 = 0xE0;
          r2 = 0x01;
          registri();
          break;
        }
    }
  }
  return false;
}

#define ntessere 14
int tag[3][ntessere] {  //avanti - indietro - destra - sinistra - luci_dx - luci_sx - luci_davanti - luci_dietro
  {5, 119, 52, 213, 87, 165, 103, 36, 0, 0, 0, 0, 0, 0},
  {7, 23, 197, 213, 148, 37, 164, 164, 24, 0, 181, 229, 132, 228}, //135
  {39, 135, 117, 21, 133, 20, 167, 0,}
};
// gira destra 45° 69 71 84
// gira sinistra 45° 85 101 100
//gira sinistra 360° 116 149 215

int Easyino_Robot::codice_tessera() {

  // for (byte i = 0; i < uid.size; i++) {
  // codice += uid.uidByte[0] < 0x10 ? " 0" : " ";
  int  codice = uid.uidByte[0];
  // }

  PICC_HaltA();
  PCD_StopCrypto1();
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < ntessere; c++) {
      if (tag[r][c] == codice) {
        switch (c) {
          case TARATURA: {
              Serial.print("Tessera n° ");
              Serial.print(codice);
              Serial.print(" -> ");
              Serial.println(c);
              return c;
              break;
            }
          case TARAAVANTI: {
              Serial.print("Tessera n° ");
              Serial.print(codice);
              Serial.print(" -> ");
              Serial.println(c);
              tempo_inizio = millis();
              vaiAvantiTaratura();
              while (!riceve_qualcosa()) {}
              tempo_tarato = (millis() - tempo_inizio);
              ferma_motori();
              kcm = tempo_tarato / 200;
              codice = 0;
              EEPROM.update(0, kcm);
              delay(1000);
              if (kcm > 255) {
                animazioneTagRiconosciuto(2);
                Serial.println("Errore taratura: costante troppo grande per la EEPROM");
              }
              else {
                Serial.println(tempo_tarato);
              }
              codice = -1;
              return c;
              break;
            }
          case TARAGIRA: {
              Serial.print("Tessera n° ");
              Serial.print(codice);
              Serial.print(" -> ");
              Serial.println(c);
              tempo_inizio = millis();
              giraDestraTaratura();
              while (!riceve_qualcosa()) {}
              tempo_tarato = (millis() - tempo_inizio);
              ferma_motori();
              kgr = tempo_tarato / 360;
              codice = 0;
              EEPROM.update(1, kgr);
              delay(1000);
              if (kcm > 255) {
                animazioneTagRiconosciuto(2);
                Serial.println("Errore taratura: costante troppo grande per la EEPROM");
              }
              else {
                Serial.println(tempo_tarato);
              }
              codice = -1;
              return c;
              break;
            }
          case PIUDESTRA: {
              Serial.print("Tessera n° ");
              Serial.print(codice);
              Serial.print(" -> ");
              Serial.println(c);
              if (ksx < 255) {
                ksx++;
                EEPROM.write(3, ksx); //coeficente giro per ruota destra
              }
              else {
                kdx--;
                EEPROM.write(2, kdx); //coeficente giro per ruota destra
              }
              return c;
              break;
            }
          case PIUSINISTRA: {
              Serial.print("Tessera n° ");
              Serial.print(codice);
              Serial.print(" -> ");
              Serial.println(c);
              if (kdx < 255) {
                kdx++;
                EEPROM.write(2, kdx); //coeficente giro per ruota destra
              }
              else {
                ksx--;
                EEPROM.write(3, ksx); //coeficente giro per ruota destra
              }
              return c;
              break;
            }
          default : {
              Serial.print("Tessera n° ");
              Serial.print(codice);
              Serial.print(" -> ");
              Serial.println(c);
              return c;
              break;
            }
        }
      }
    }
  }
  Serial.print("Tessera n° ");
  Serial.println(codice);
  return codice;
}


void Easyino_Robot::vaiAvantiTaratura() {
  digitalWrite(A1, HIGH);
  digitalWrite(A3, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A4, LOW);
  analogWrite(3, kdx); // Ruota destra
  analogWrite(6, ksx); // Ruota sinistra
}

void Easyino_Robot::giraDestraTaratura() {
  digitalWrite(A1, HIGH);
  digitalWrite(A3, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A4, LOW);
  analogWrite(3, kdx); // Ruota destra
  analogWrite(6, ksx); // Ruota sinistra
}






void animazione(int com, int durata, int vel) {
  if (com != -1) {
    int camb = lunghezza(com);
    for (long int d = 0, v = 1; d < durata; d++) {
      if (d % vel == (int)((vel - 1) * v / camb)) {
        r1 = anim[com][(v - 1) * 2];
        r2 = anim[com][((v - 1) * 2) + 1];
        registri();
        v = (v + 1) % (camb + 1);
        if (v == 0) {
          v = 1;
        }
      }
      delay(1);
    }

  }
  else {
    delay(durata);
  }
  if (com != 4) {
    spegni_led();
    ferma_motori();
  }
  else{
  r1 = 0;
  r2 = 0;
  registri();
  }
  Serial.print("Animazione n° ");
  Serial.println(an);
}
void Easyino_Robot::accendiFrecciaDestra() {
  an = 0;
}
void Easyino_Robot::accendiFrecciaSinistra() {
  an = 1;
}
void Easyino_Robot::luciFrontali() {
  an = 2;
}
void Easyino_Robot::luciPosteriori() {
  an = 3;
}
void Easyino_Robot::animazioneTagRiconosciuto(int cicli) {
  animazione(4, duranim[4], duranim[4]* cicli);
}
void Easyino_Robot::animazioneTagRiconosciuto() {
  animazioneTagRiconosciuto(1);
}



void Easyino_Robot::vaiAvanti(int centimetri) {
  if (isEasy)luciFrontali();
  digitalWrite(A1, HIGH);
  digitalWrite(A3, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A4, LOW);

  analogWrite(3, kdx); // Ruota destra
  analogWrite(6, ksx); // Ruota sinistra
  animazione(an, (int)(kcm * centimetri ), duranim[an]);
}
void Easyino_Robot::vaiAvanti() {
  vaiAvanti(100);
}
void Easyino_Robot::vaiIndietro(int centimetri) {
  if (isEasy)luciPosteriori();
  digitalWrite(A1, LOW);
  digitalWrite(A3, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A4, HIGH);
  analogWrite(3, kdx); // Ruota destra
  analogWrite(6, ksx); // Ruota sinistra
  animazione(an, (int)(kcm * centimetri ), duranim[an]);
}
void Easyino_Robot::vaiIndietro() {
  vaiIndietro(100);
}
void Easyino_Robot::giraDestra(int gradi) {
  if (isEasy)accendiFrecciaDestra();
  digitalWrite(A1, HIGH);
  digitalWrite(A3, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A4, LOW);
  analogWrite(3, kdx); // Ruota destra
  analogWrite(6, ksx); // Ruota sinistra
  animazione(an, (int)(kgr * gradi ), duranim[an]);
}
void Easyino_Robot::giraDestra() {
  giraDestra(90);
}
void Easyino_Robot::giraSinistra(int gradi) {
  if (isEasy)accendiFrecciaSinistra();
  digitalWrite(A1, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A4, LOW);
  analogWrite(3, kdx); // Ruota destra
  analogWrite(6, ksx); // Ruota sinistra
  animazione(an, (int)(kgr * gradi ), duranim[an]);
}
void Easyino_Robot::giraSinistra() {
  giraSinistra(90);
}



void Easyino_Robot::avanti(int centimetri) {
  luciFrontali();
  vaiAvanti(centimetri);
}
void Easyino_Robot::indietro(int centimetri) {
  luciPosteriori();
  vaiIndietro(centimetri);
}
void Easyino_Robot::destra(int gradi) {
  accendiFrecciaDestra();
  giraDestra(gradi);
}
void Easyino_Robot::sinistra(int gradi) {
  accendiFrecciaSinistra();
  giraSinistra(gradi);
}
