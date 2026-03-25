#ifndef ZEPHYR_DRIVERS_SENSOR_H3LIS331DL_REG_H_
#define ZEPHYR_DRIVERS_SENSOR_H3LIS331DL_REG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * REGISTER MAP 
 * ============================================================ */

/* Reserved: 0x00 - 0x0E  (do NOT write) */
#define H3LIS331DL_REG_WHO_AM_I         0x0F  /* R    - Device ID, expected 0x32          */
/* Reserved: 0x10 - 0x1F  (do NOT write) */
#define H3LIS331DL_REG_CTRL_REG1        0x20  /* R/W  - Power/ODR/axis enable             */
#define H3LIS331DL_REG_CTRL_REG2        0x21  /* R/W  - High-pass filter control          */
#define H3LIS331DL_REG_CTRL_REG3        0x22  /* R/W  - Interrupt pin control             */
#define H3LIS331DL_REG_CTRL_REG4        0x23  /* R/W  - Full-scale / BDU / SPI mode       */
#define H3LIS331DL_REG_CTRL_REG5        0x24  /* R/W  - Sleep-to-wake                     */
#define H3LIS331DL_REG_HP_FILTER_RESET  0x25  /* R    - Dummy: reading resets HP filter   */
#define H3LIS331DL_REG_REFERENCE        0x26  /* R/W  - HP filter reference value         */
#define H3LIS331DL_REG_STATUS_REG       0x27  /* R    - Data-ready / overrun flags        */
#define H3LIS331DL_REG_OUT_X_L          0x28  /* R    - X-axis LSB                        */
#define H3LIS331DL_REG_OUT_X_H          0x29  /* R    - X-axis MSB                        */
#define H3LIS331DL_REG_OUT_Y_L          0x2A  /* R    - Y-axis LSB                        */
#define H3LIS331DL_REG_OUT_Y_H          0x2B  /* R    - Y-axis MSB                        */
#define H3LIS331DL_REG_OUT_Z_L          0x2C  /* R    - Z-axis LSB                        */
#define H3LIS331DL_REG_OUT_Z_H          0x2D  /* R    - Z-axis MSB                        */
/* Reserved: 0x2E - 0x2F  (do NOT write) */
#define H3LIS331DL_REG_INT1_CFG         0x30  /* R/W  - Interrupt 1 configuration        */
#define H3LIS331DL_REG_INT1_SRC         0x31  /* R    - Interrupt 1 source (read-only)   */
#define H3LIS331DL_REG_INT1_THS         0x32  /* R/W  - Interrupt 1 threshold            */
#define H3LIS331DL_REG_INT1_DURATION    0x33  /* R/W  - Interrupt 1 duration             */
#define H3LIS331DL_REG_INT2_CFG         0x34  /* R/W  - Interrupt 2 configuration        */
#define H3LIS331DL_REG_INT2_SRC         0x35  /* R    - Interrupt 2 source (read-only)   */
#define H3LIS331DL_REG_INT2_THS         0x36  /* R/W  - Interrupt 2 threshold            */
#define H3LIS331DL_REG_INT2_DURATION    0x37  /* R/W  - Interrupt 2 duration             */
/* Reserved: 0x38 - 0x3F  (do NOT write) */

/* ============================================================
 * WHO_AM_I  
 * ============================================================ */
#define H3LIS331DL_DEVICE_ID            0x32  /* 0b00110010 */

/* ============================================================
 * CTRL_REG1 (0x20
 * ============================================================ */

/* Power mode  */
#define H3LIS331DL_PM_POWER_DOWN        0b00000000  /* 000 << 5 */
#define H3LIS331DL_PM_NORMAL            0b00100000  /* 001 << 5 */
#define H3LIS331DL_PM_LOW_POWER_0_5HZ   0b01000000  /* 010 << 5 - ODR = 0.5 Hz */
#define H3LIS331DL_PM_LOW_POWER_1HZ     0b01100000  /* 011 << 5 - ODR = 1   Hz */
#define H3LIS331DL_PM_LOW_POWER_2HZ     0b10000000  /* 100 << 5 - ODR = 2   Hz */
#define H3LIS331DL_PM_LOW_POWER_5HZ     0b10100000  /* 101 << 5 - ODR = 5   Hz */
#define H3LIS331DL_PM_LOW_POWER_10HZ    0b11000000  /* 110 << 5 - ODR = 10  Hz */
#define H3LIS331DL_PM_MASK              0b11100000

/* Output data rate  */
#define H3LIS331DL_DR_50HZ              0b00000000  /* 00 << 3 - LP cutoff  37 Hz */
#define H3LIS331DL_DR_100HZ             0b00001000  /* 01 << 3 - LP cutoff  74 Hz */
#define H3LIS331DL_DR_400HZ             0b00010000  /* 10 << 3 - LP cutoff 292 Hz */
#define H3LIS331DL_DR_1000HZ            0b00011000  /* 11 << 3 - LP cutoff 780 Hz */
#define H3LIS331DL_DR_MASK              0b00011000

/* Axis enable bits */
#define H3LIS331DL_ZEN                  0b00000100  /* Z-axis enable (default: 1) */
#define H3LIS331DL_YEN                  0b00000010  /* Y-axis enable (default: 1) */
#define H3LIS331DL_XEN                  0b00000001  /* X-axis enable (default: 1) */
#define H3LIS331DL_ALL_AXES_EN          0b00000111

/* Default value after power-on: normal mode, 50 Hz, all axes enabled */
#define H3LIS331DL_CTRL_REG1_DEFAULT    0b00000111

/* ============================================================
 * CTRL_REG2
 * ============================================================ */
#define H3LIS331DL_BOOT                 0b10000000 
#define H3LIS331DL_HPM_NORMAL           0b00000000  
#define H3LIS331DL_HPM_REFERENCE        0b00100000  
#define H3LIS331DL_HPM_NORMAL_ALT       0b01000000  
#define H3LIS331DL_HPM_MASK             0b01100000
#define H3LIS331DL_FDS                  0b00010000 
#define H3LIS331DL_HPEN2                0b00001000 
#define H3LIS331DL_HPEN1                0b00000100 

/* HP filter cutoff */
#define H3LIS331DL_HPCF_8               0b00000000   
#define H3LIS331DL_HPCF_16              0b00000001   
#define H3LIS331DL_HPCF_32              0b00000010   
#define H3LIS331DL_HPCF_64              0b00000011  
#define H3LIS331DL_HPCF_MASK            0b00000011

/* ============================================================
 * CTRL_REG3
 * ============================================================ */
#define H3LIS331DL_IHL_ACTIVE_HIGH      0b00000000
#define H3LIS331DL_IHL_ACTIVE_LOW       0b10000000
#define H3LIS331DL_PP_OD_PUSH_PULL      0b00000000
#define H3LIS331DL_PP_OD_OPEN_DRAIN     0b01000000
#define H3LIS331DL_LIR2                 0b00100000  /* Latch INT2 request              */
#define H3LIS331DL_LIR1                 0b00000100  /* Latch INT1 request              */

/* INT pin signal source - Table 19 */
#define H3LIS331DL_I_CFG_INT_SRC        0b00  /* Interrupt own source            */
#define H3LIS331DL_I_CFG_INT1_OR_INT2   0b01  /* INT1 OR INT2                    */
#define H3LIS331DL_I_CFG_DATA_READY     0b10  /* Data ready                      */
#define H3LIS331DL_I_CFG_BOOT_RUNNING   0b11  /* Boot running                    */

/* INT2 signal select (bits [4:3]) */
#define H3LIS331DL_I2_CFG_MASK          0b00011000
#define H3LIS331DL_I2_CFG_INT_SRC       (H3LIS331DL_I_CFG_INT_SRC      << 3)
#define H3LIS331DL_I2_CFG_INT1_OR_INT2  (H3LIS331DL_I_CFG_INT1_OR_INT2 << 3)
#define H3LIS331DL_I2_CFG_DATA_READY    (H3LIS331DL_I_CFG_DATA_READY   << 3)
#define H3LIS331DL_I2_CFG_BOOT_RUNNING  (H3LIS331DL_I_CFG_BOOT_RUNNING << 3)

/* INT1 signal select (bits [1:0]) */
#define H3LIS331DL_I1_CFG_MASK          0b00000011
#define H3LIS331DL_I1_CFG_INT_SRC       (H3LIS331DL_I_CFG_INT_SRC      << 0)
#define H3LIS331DL_I1_CFG_INT1_OR_INT2  (H3LIS331DL_I_CFG_INT1_OR_INT2 << 0)
#define H3LIS331DL_I1_CFG_DATA_READY    (H3LIS331DL_I_CFG_DATA_READY   << 0)
#define H3LIS331DL_I1_CFG_BOOT_RUNNING  (H3LIS331DL_I_CFG_BOOT_RUNNING << 0)

/* ============================================================
 * CTRL_REG4
 * ============================================================ */
#define H3LIS331DL_BDU                  0b10000000  /* 1 = block update until MSB+LSB read */
#define H3LIS331DL_BLE_LITTLE_ENDIAN    0b00000000  /* LSB at lower address (default)  */
#define H3LIS331DL_BLE_BIG_ENDIAN       0b01000000  /* MSB at lower address            */

/* Full-scale selection  */
#define H3LIS331DL_FS_100G              0b00000000  /* 00 << 4 - ±100 g, sens = 49  mg/digit */
#define H3LIS331DL_FS_200G              0b00010000  /* 01 << 4 - ±200 g, sens = 98  mg/digit */
#define H3LIS331DL_FS_400G              0b00110000  /* 11 << 4 - ±400 g, sens = 195 mg/digit */
#define H3LIS331DL_FS_MASK              0b00110000

#define H3LIS331DL_SIM_4WIRE            0b00000000  /* 4-wire SPI (default)            */
#define H3LIS331DL_SIM_3WIRE            0b00000001  /* 3-wire SPI                      */

/* ============================================================
 
 * ============================================================ */
#define H3LIS331DL_SLEEP_TO_WAKE_DIS    0b00000000  /* 00 - disabled                   */
#define H3LIS331DL_SLEEP_TO_WAKE_EN     0b00000011  /* 11 - enabled                    */
#define H3LIS331DL_SLEEP_TO_WAKE_MASK   0b00000011

/* ============================================================
 * Data status check
 * ============================================================ */
#define H3LIS331DL_ZYXOR                0b10000000  /* XYZ overrun                     */
#define H3LIS331DL_ZOR                  0b01000000  /* Z overrun                       */
#define H3LIS331DL_YOR                  0b00100000  /* Y overrun                       */
#define H3LIS331DL_XOR                  0b00010000  /* X overrun                       */
#define H3LIS331DL_ZYXDA                0b00001000  /* XYZ new data available           */
#define H3LIS331DL_ZDA                  0b00000100  /* Z new data available             */
#define H3LIS331DL_YDA                  0b00000010  /* Y new data available             */
#define H3LIS331DL_XDA                  0b00000001  /* X new data available             */

/* ============================================================
 * Interrupt
 * ============================================================ */
#define H3LIS331DL_INT_AOI_OR           0b00000000  /* OR  combination                 */
#define H3LIS331DL_INT_AOI_AND          0b10000000  /* AND combination                 */
#define H3LIS331DL_INT_ZHIE             0b00100000  /* Z high interrupt enable         */
#define H3LIS331DL_INT_ZLIE             0b00010000  /* Z low  interrupt enable         */
#define H3LIS331DL_INT_YHIE             0b00001000  /* Y high interrupt enable         */
#define H3LIS331DL_INT_YLIE             0b00000100  /* Y low  interrupt enable         */
#define H3LIS331DL_INT_XHIE             0b00000010  /* X high interrupt enable         */
#define H3LIS331DL_INT_XLIE             0b00000001  /* X low  interrupt enable         */
#define H3LIS331DL_INT_ALL_HIGH         0b00101010  /* All high events                 */
#define H3LIS331DL_INT_ALL_LOW          0b00010101  /* All low  events                 */

/* ============================================================
 * Interrupt active flag
 * ============================================================ */
#define H3LIS331DL_INT_SRC_IA           0b01000000 
#define H3LIS331DL_INT_SRC_ZH           0b00100000
#define H3LIS331DL_INT_SRC_ZL           0b00010000
#define H3LIS331DL_INT_SRC_YH           0b00001000
#define H3LIS331DL_INT_SRC_YL           0b00000100
#define H3LIS331DL_INT_SRC_XH           0b00000010
#define H3LIS331DL_INT_SRC_XL           0b00000001

/* ============================================================
 * SPI bus protocol helpers
 * ============================================================ */
#define H3LIS331DL_SPI_READ             0x80  /* bit 7 of first SPI byte = RW=1  */
#define H3LIS331DL_SPI_WRITE            0x00  /* bit 7 of first SPI byte = RW=0  */
#define H3LIS331DL_SPI_AUTO_INC         0x40  /* bit 6 of first SPI byte = MS=1  */

/* ============================================================
 * I2C address variants (Section 6.1.1)
 *   SA0=0 (SDO/SA0 tied to GND) -> 0x18
 *   SA0=1 (SDO/SA0 tied to Vdd) -> 0x19
 * ============================================================ */
#define H3LIS331DL_I2C_ADDR_SA0_LOW     0x18
#define H3LIS331DL_I2C_ADDR_SA0_HIGH    0x19

/* I2C multi-byte read: MSB of sub-address must be set (Section 6.1) */
#define H3LIS331DL_I2C_AUTO_INC         0x80

/* ============================================================
 * Sensitivity constants (mg/digit) - Table 2
 * ============================================================ */
#define H3LIS331DL_SENS_100G_MG_PER_DIGIT   49
#define H3LIS331DL_SENS_200G_MG_PER_DIGIT   98
#define H3LIS331DL_SENS_400G_MG_PER_DIGIT  195

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_H3LIS331DL_REG_H_ */