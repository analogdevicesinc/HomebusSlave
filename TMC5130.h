/** \file TMC5130.h ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: TMC5130.h
 *         Description: TMC5130 interface
 *
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#ifndef __TMC5130_H
#define __TMC5130_H

//Registers
#define TMC5130_GCONF        0x00
#define TMC5130_GSTAT        0x01
#define TMC5130_IFCNT        0x02
#define TMC5130_SLAVECONF    0x03
#define TMC5130_IOIN         0x04
#define TMC5130_X_COMPARE    0x05


#define TMC5130_IHOLD_IRUN   0x10
#define TMC5130_TPOWERDOWN   0x11
#define TMC5130_TSTEP        0x12
#define TMC5130_TPWMTHRS     0x13
#define TMC5130_TCOOLTHRS    0x14
#define TMC5130_THIGH        0x15
#define TMC5130_RAMPMODE     0x20
#define TMC5130_XACTUAL      0x21
#define TMC5130_VACTUAL      0x22
#define TMC5130_VSTART       0x23
#define TMC5130_A1           0x24
#define TMC5130_V1           0x25
#define TMC5130_AMAX         0x26
#define TMC5130_VMAX         0x27
#define TMC5130_DMAX         0x28
#define TMC5130_D1           0x2A
#define TMC5130_VSTOP        0x2B
#define TMC5130_TZEROWAIT    0x2C
#define TMC5130_XTARGET      0x2D
#define TMC5130_VDCMIN       0x33
#define TMC5130_SWMODE       0x34
#define TMC5130_RAMPSTAT     0x35
#define TMC5130_XLATCH       0x36
#define TMC5130_ENCMODE      0x38
#define TMC5130_XENC         0x39
#define TMC5130_ENC_CONST    0x3A
#define TMC5130_ENC_STATUS   0x3B
#define TMC5130_ENC_LATCH    0x3C
#define TMC5130_MSLUT0       0x60
#define TMC5130_MSLUT1       0x61
#define TMC5130_MSLUT2       0x62
#define TMC5130_MSLUT3       0x63
#define TMC5130_MSLUT4       0x64
#define TMC5130_MSLUT5       0x65
#define TMC5130_MSLUT6       0x66
#define TMC5130_MSLUT7       0x67
#define TMC5130_MSLUTSEL     0x68
#define TMC5130_MSLUTSTART   0x69
#define TMC5130_MSCNT        0x6A
#define TMC5130_MSCURACT     0x6B
#define TMC5130_CHOPCONF     0x6C
#define TMC5130_COOLCONF     0x6D
#define TMC5130_DCCTRL       0x6E
#define TMC5130_DRVSTATUS    0x6F
#define TMC5130_PWMCONF      0x70
#define TMC5130_PWMSCALE     0x71
#define TMC5130_ENCM_CTRL    0x72
#define TMC5130_LOST_STEPS   0x73

#define WHICH_5130(m)       (m)

//Write bit
#define TMC5130_WRITE        0x80

//Ramp modes (Register TMC5130_RAMPMODE)
#define TMC5130_MODE_POSITION   0
#define TMC5130_MODE_VELPOS     1
#define TMC5130_MODE_VELNEG     2
#define TMC5130_MODE_HOLD       3

//Configuration bits (Register TMC5130_GCONF)
#define TMC5130_GCONF_ISCALE_ANALOG     0x00001
#define TMC5130_GCONF_INT_RSENSE        0x00002
#define TMC5130_GCONF_ENC_COMMUTATION   0x00008

#define TMC5130_GCONF_EN_PWM_MODE       0x00004
#define TMC5130_GCONF_SHAFT             0x00010
#define TMC5130_GCONF_DIAG0_ERROR       0x00020
#define TMC5130_GCONF_DIAG0_OTPW        0x00040
#define TMC5130_GCONF_DIAG0_STALL_STEP  0x00080
#define TMC5130_GCONF_DIAG1_STALL_DIR   0x00100
#define TMC5130_DIAG1_INDEX             0x00200
#define TMC5130_DIAG1_ONSTATE           0x00400
#define TMC5130_DIAG1_STEPS_SKIPPED     0x00800
#define TMC5130_GCONF_DIAG0_PUSHPULL    0x01000
#define TMC5130_GCONF_DIAG1_PUSHPULL    0x02000
#define TMC5130_GCONF_SMALL_HYSTERESIS  0x04000
#define TMC5130_GCONF_STOP_ENABLE       0x08000
#define TMC5130_GCONF_DIRECT_MODE       0x10000
#define TMC5130_GCONF_TEST_MODE         0x20000

//End switch mode bits (Register TMC5130_SWMODE)
#define TMC5130_SW_STOPL_ENABLE   0x0001
#define TMC5130_SW_STOPR_ENABLE   0x0002
#define TMC5130_SW_STOPL_POLARITY 0x0004
#define TMC5130_SW_STOPR_POLARITY 0x0008
#define TMC5130_SW_SWAP_LR        0x0010
#define TMC5130_SW_LATCH_L_ACT    0x0020
#define TMC5130_SW_LATCH_L_INACT  0x0040
#define TMC5130_SW_LATCH_R_ACT    0x0080
#define TMC5130_SW_LATCH_R_INACT  0x0100
#define TMC5130_SW_LATCH_ENC      0x0200
#define TMC5130_SW_SG_STOP        0x0400
#define TMC5130_SW_SOFTSTOP       0x0800

//Status bits
#define TMC5130_RS_STOPL          0x0001
#define TMC5130_RS_STOPR          0x0002
#define TMC5130_RS_LATCHL         0x0004
#define TMC5130_RS_LATCHR         0x0008
#define TMC5130_RS_EV_STOPL       0x0010
#define TMC5130_RS_EV_STOPR       0x0020
#define TMC5130_RS_EV_STOP_SG     0x0040
#define TMC5130_RS_EV_POSREACHED  0x0080
#define TMC5130_RS_VELREACHED     0x0100
#define TMC5130_RS_POSREACHED     0x0200
#define TMC5130_RS_VZERO          0x0400
#define TMC5130_RS_ZEROWAIT       0x0800
#define TMC5130_RS_SECONDMOVE     0x1000
#define TMC5130_RS_SG             0x2000

//Encoder mode bits (Register TMC5130_ENCMODE)
#define TMC5130_EM_DECIMAL        0x0400
#define TMC5130_EM_LATCH_XACT     0x0200
#define TMC5130_EM_CLR_XENC       0x0100
#define TMC5130_EM_NEG_EDGE       0x0080
#define TMC5130_EM_POS_EDGE       0x0040
#define TMC5130_EM_CLR_ONCE       0x0020
#define TMC5130_EM_CLR_CONT       0x0010
#define TMC5130_EM_IGNORE_AB      0x0008
#define TMC5130_EM_POL_N          0x0004
#define TMC5130_EM_POL_B          0x0002
#define TMC5130_EM_POL_A          0x0001

#define TPOWERDOWN_FACTOR (4.17792*100.0/255.0)

void WriteTMC5130Datagram(uint8_t Which562, uint8_t Address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
void WriteTMC5130Int(uint8_t Which562, uint8_t Address, int Value);
int ReadTMC5130Int(uint8_t Which562, uint8_t Address);
void SetTMC5130ChopperTOff(uint8_t Motor, uint8_t TOff);
void SetTMC5130ChopperHysteresisStart(uint8_t Motor, uint8_t HysteresisStart);
void SetTMC5130ChopperHysteresisEnd(uint8_t Motor, uint8_t HysteresisEnd);
void SetTMC5130ChopperBlankTime(uint8_t Motor, uint8_t BlankTime);
void SetTMC5130ChopperSync(uint8_t Motor, uint8_t Sync);
void SetTMC5130ChopperMStepRes(uint8_t Motor, uint8_t MRes);
void SetTMC5130ChopperDisableShortToGround(uint8_t Motor, uint8_t Disable);
void SetTMC5130ChopperVHighChm(uint8_t Motor, uint8_t VHighChm);
void SetTMC5130ChopperVHighFs(uint8_t Motor, uint8_t VHighFs);
void SetTMC5130ChopperConstantTOffMode(uint8_t Motor, uint8_t ConstantTOff);
void SetTMC5130ChopperRandomTOff(uint8_t Motor, uint8_t RandomTOff);
void SetTMC5130ChopperDisableFastDecayComp(uint8_t Motor, uint8_t Disable);
void SetTMC5130ChopperFastDecayTime(uint8_t Motor, uint8_t Time);
void SetTMC5130ChopperSineWaveOffset(uint8_t Motor, uint8_t Offset);
uint8_t GetTMC5130ChopperTOff(uint8_t Motor);
uint8_t GetTMC5130ChopperHysteresisStart(uint8_t Motor);
uint8_t GetTMC5130ChopperHysteresisEnd(uint8_t Motor);
uint8_t GetTMC5130ChopperBlankTime(uint8_t Motor);
uint8_t GetTMC5130ChopperSync(uint8_t Motor);
uint8_t GetTMC5130ChopperMStepRes(uint8_t Motor);
uint8_t GetTMC5130ChopperDisableShortToGround(uint8_t Motor);
uint8_t GetTMC5130ChopperVHighChm(uint8_t Motor);
uint8_t GetTMC5130ChopperVHighFs(uint8_t Motor);
uint8_t GetTMC5130ChopperConstantTOffMode(uint8_t Motor);
uint8_t GetTMC5130ChopperRandomTOff(uint8_t Motor);
uint8_t GetTMC5130ChopperDisableFastDecayComp(uint8_t Motor);
uint8_t GetTMC5130ChopperFastDecayTime(uint8_t Motor);
uint8_t GetTMC5130ChopperSineWaveOffset(uint8_t Motor);
void SetTMC5130SmartEnergyUpStep(uint8_t Motor, uint8_t UpStep);
void SetTMC5130SmartEnergyDownStep(uint8_t Motor, uint8_t DownStep);
void SetTMC5130SmartEnergyStallLevelMax(uint8_t Motor, uint8_t Max);
void SetTMC5130SmartEnergyStallLevelMin(uint8_t Motor, uint8_t Min);
void SetTMC5130SmartEnergyStallThreshold(uint8_t Motor, char Threshold);
void SetTMC5130SmartEnergyIMin(uint8_t Motor, uint8_t IMin);
void SetTMC5130SmartEnergyFilter(uint8_t Motor, uint8_t Filter);
uint8_t GetTMC5130SmartEnergyUpStep(uint8_t Motor);
uint8_t GetTMC5130SmartEnergyDownStep(uint8_t Motor);
uint8_t GetTMC5130SmartEnergyStallLevelMax(uint8_t Motor);
uint8_t GetTMC5130SmartEnergyStallLevelMin(uint8_t Motor);
int GetTMC5130SmartEnergyStallThreshold(uint8_t Motor);
uint8_t GetTMC5130SmartEnergyIMin(uint8_t Motor);
uint8_t GetTMC5130SmartEnergyFilter(uint8_t Motor);
void SetTMC5130PWMFreewheelMode(uint8_t Motor, uint8_t Mode);
void SetTMC5130PWMSymmetric(uint8_t Motor, uint8_t Symmetric);
void SetTMC5130PWMAutoscale(uint8_t Motor, uint8_t Autoscale);
void SetTMC5130PWMFrequency(uint8_t Motor, uint8_t Frequency);
void SetTMC5130PWMGrad(uint8_t Motor, uint8_t PWMGrad);
void SetTMC5130PWMAmpl(uint8_t Motor, uint8_t PWMAmpl);
uint8_t GetTMC5130PWMFreewheelMode(uint8_t Motor);
uint8_t GetTMC5130PWMSymmetric(uint8_t Motor);
uint8_t GetTMC5130PWMAutoscale(uint8_t Motor);
uint8_t GetTMC5130PWMFrequency(uint8_t Motor);
uint8_t GetTMC5130PWMGrad(uint8_t Motor);
uint8_t GetTMC5130PWMAmpl(uint8_t Motor);

void Read5130State(uint8_t Which5130, uint32_t *StallGuard, uint8_t *SmartEnergy, uint8_t *Flags);
void InitMotorDrivers(void);
void DisableTMC5130(uint8_t Motor);
void EnableTMC5130(uint8_t Motor);

void HardStop(uint32_t Motor);

int ConvertVelocityUserToInternal(int UserVelocity);
int ConvertAccelerationUserToInternal(int UserAcceleration);
int ConvertVelocityInternalToUser(int InternalVelocity);
int ConvertAccelerationInternalToUser(int InternalAcceleration);
int ConvertInternalToInternal(int Internal);

#endif
