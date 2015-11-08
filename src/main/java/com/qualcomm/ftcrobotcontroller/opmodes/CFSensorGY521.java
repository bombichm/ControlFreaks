//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cController.I2cPortReadyCallback;
import com.qualcomm.robotcore.util.RobotLog;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.locks.Lock;

public class CFSensorGY521 implements HardwareDevice, I2cPortReadyCallback {
    public static final int ADDRESS_I2C = 0x68;

    protected static final int OFFSET_RAW_X_VAL = 0x06;
    protected static final int OFFSET_RAW_Y_VAL = 0x08;
    protected static final int OFFSET_RAW_Z_VAL = 0x0A;
    protected static final int OFFSET_USR_X_VAL = 0x13;
    protected static final int OFFSET_USR_Y_VAL = 0x15;
    protected static final int OFFSET_USR_Z_VAL = 0x17;
    protected static final int MPU6050_RA_PWR_MGMT_1 = 0x6B;
    protected static final int MPU6050_PWR1_CLKSEL_BIT = 2;
    protected static final int MPU6050_PWR1_CLKSEL_LENGTH = 3;

    private final DeviceInterfaceModule dim;
    private final byte[] I2cReadCache;
    private final Lock I2cReadCacheLock;
    private final byte[] I2cWriteCache;
    private final Lock I2cWriteCacheLock;
    private final int dimI2cPort;
    protected static final int I2CBufferLength = 0x16;
    private GyroData lastData;

    private boolean inited = false;
    private boolean gotData = false;
    public CFSensorGY521(DeviceInterfaceModule deviceInterfaceModule, int physicalPort) {
        this.lastData = new GyroData();
        this.dim = deviceInterfaceModule;
        this.dimI2cPort = physicalPort;
        this.I2cReadCache = deviceInterfaceModule.getI2cReadCache(physicalPort);
        this.I2cReadCacheLock = deviceInterfaceModule.getI2cReadCacheLock(physicalPort);
        this.I2cWriteCache = deviceInterfaceModule.getI2cWriteCache(physicalPort);
        this.I2cWriteCacheLock = deviceInterfaceModule.getI2cWriteCacheLock(physicalPort);
        //int physicalPort, int i2cAddress, int memAddress, int length
        deviceInterfaceModule.enableI2cReadMode(physicalPort, ADDRESS_I2C, 0, I2CBufferLength);
        deviceInterfaceModule.setI2cPortActionFlag(physicalPort);
        deviceInterfaceModule.writeI2cCacheToController(physicalPort);
        deviceInterfaceModule.registerForI2cPortReadyCallback(this, physicalPort);

        //we need to read the Ic2Buffer
        //setClockSource(MPU6050_CLOCK_PLL_XGYRO);
        //setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        //setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        //setSleepEnabled(false);
        initMPU6050();
        inited = true;
    }

    private void initMPU6050(){
//        #define MPU6050_PWR1_DEVICE_RESET_BIT   7
//        #define MPU6050_PWR1_SLEEP_BIT          6
//        #define MPU6050_PWR1_CYCLE_BIT          5
//        #define MPU6050_PWR1_TEMP_DIS_BIT       3
//        #define MPU6050_PWR1_CLKSEL_BIT         2
//        #define MPU6050_PWR1_CLKSEL_LENGTH      3
        this.dim.enableI2cWriteMode(dimI2cPort, ADDRESS_I2C, MPU6050_RA_PWR_MGMT_1, 1);

        try {
            this.I2cWriteCacheLock.lock();
            this.I2cWriteCache[MPU6050_RA_PWR_MGMT_1] = 1;
        } finally {
            this.I2cWriteCacheLock.unlock();
        }

        this.dim.setI2cPortActionFlag(this.dimI2cPort);
        this.dim.writeI2cCacheToController(this.dimI2cPort);
    }

    /** Set clock source setting.
     * An internal 8MHz oscillator, gyroscope based clock, or external sources can
     * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
     * or an external source is chosen as the clock source, the MPU-60X0 can operate
     * in low power modes with the gyroscopes disabled.
     *
     * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
     * However, it is highly recommended that the device be configured to use one of
     * the gyroscopes (or an external clock source) as the clock reference for
     * improved stability. The clock source can be selected according to the following table:
     *
     * <pre>
     * CLK_SEL | Clock Source
     * --------+--------------------------------------
     * 0       | Internal oscillator
     * 1       | PLL with X Gyro reference
     * 2       | PLL with Y Gyro reference
     * 3       | PLL with Z Gyro reference
     * 4       | PLL with external 32.768kHz reference
     * 5       | PLL with external 19.2MHz reference
     * 6       | Reserved
     * 7       | Stops the clock and keeps the timing generator in reset
     * </pre>
     *
     * @param source New clock source setting
     * //@see getClockSource()
     * //@see MPU6050_RA_PWR_MGMT_1
     * //@see MPU6050_PWR1_CLKSEL_BIT
     * //@see MPU6050_PWR1_CLKSEL_LENGTH
     */
    void setClockSource(byte source) {
        //I2Cdev::writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
        //int port ,int i2cAddress, int memAddress, int length
//        this.dim.enableI2cWriteMode(dimI2cPort, ADDRESS_I2C, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_LENGTH);
//
//        try {
//            this.I2cWriteCacheLock.lock();
//
//            this.I2cWriteCache[MPU6050_PWR1_CLKSEL_BIT] = source;
//        } finally {
//            this.I2cWriteCacheLock.unlock();
//        }
//
//        this.dim.setI2cPortActionFlag(this.dimI2cPort);
//        this.dim.writeI2cCacheToController(this.dimI2cPort);
    }

    /** Set full-scale gyroscope range.
     * @param range New full-scale gyroscope range value
     * //@see getFullScaleRange()
     * //@see MPU6050_GYRO_FS_250
     * //@see MPU6050_RA_GYRO_CONFIG
     * //@see MPU6050_GCONFIG_FS_SEL_BIT
     * //@see MPU6050_GCONFIG_FS_SEL_LENGTH
     */
    void setFullScaleGyroRange(byte range) {
        //I2Cdev::writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
//        this.dim.enableI2cWriteMode(dimI2cPort, ADDRESS_I2C, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_LENGTH);
//
//        try {
//            this.I2cWriteCacheLock.lock();
//            this.I2cWriteCache[MPU6050_GCONFIG_FS_SEL_BIT] = range;
//        } finally {
//            this.I2cWriteCacheLock.unlock();
//        }
//
//        this.dim.setI2cPortActionFlag(this.dimI2cPort);
//        this.dim.writeI2cCacheToController(this.dimI2cPort);
    }

    /** Get sleep mode status.
     * Setting the SLEEP bit in the register puts the device into very low power
     * sleep mode. In this mode, only the serial interface and internal registers
     * remain active, allowing for a very low standby current. Clearing this bit
     * puts the device back into normal mode. To save power, the individual standby
     * selections for each of the gyros should be used if any gyro axis is not used
     * by the application.
     * @return Current sleep mode enabled status
     * //@see MPU6050_RA_PWR_MGMT_1
     * //@see MPU6050_PWR1_SLEEP_BIT
     */
    boolean getSleepEnabled() {
        //I2Cdev::readBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, buffer);
        //return buffer[0];
        return false;
    }
    /** Set sleep mode status.
     * @param enabled New sleep mode enabled status
     * //@see getSleepEnabled()
     * //@see MPU6050_RA_PWR_MGMT_1
     * //@see MPU6050_PWR1_SLEEP_BIT
     */
    void setSleepEnabled(boolean enabled) {
//        //I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
//        this.dim.enableI2cWriteMode(dimI2cPort, ADDRESS_I2C, MPU6050_RA_PWR_MGMT_1, 1);
//
//        try {
//            this.I2cWriteCacheLock.lock();
//            this.I2cWriteCache[MPU6050_PWR1_SLEEP_BIT] = range;
//        } finally {
//            this.e.unlock();
//        }
//
//        this.dim.setI2cPortActionFlag(this.dimI2cPort);
//        this.dim.writeI2cCacheToController(this.dimI2cPort);
    }

    public int getHeading() {
        return 0;
    }

    public double getRotation() {
        //this.notSupported();
        return 0.0D;
    }



    private class GyroData {

        short rawX;
        short rawY;
        short rawZ;
        short heading;
        private GyroData() {
        }
    }

    private GyroData readRawData() {

        try {
            this.I2cReadCacheLock.lock();
            ByteBuffer lastData = ByteBuffer.wrap(this.I2cReadCache);
            lastData.order(ByteOrder.LITTLE_ENDIAN);

            this.lastData.rawX = this.I2cReadCache[OFFSET_RAW_X_VAL];
            this.lastData.rawY = this.I2cReadCache[OFFSET_RAW_Y_VAL];
            this.lastData.rawZ = this.I2cReadCache[OFFSET_RAW_Z_VAL];
        } finally {
            this.I2cReadCacheLock.unlock();
        }
        return this.lastData;
    }

    public int rawX() {
        return 0;
    }

    public int rawY() {
        return 0;
    }

    public int rawZ() {
        return 0;
    }

    public void portIsReady(int port) {
        if(this.inited) {
            this.readRawData();
            this.gotData = true;
        }

        this.dim.readI2cCacheFromController(this.dimI2cPort);
        this.dim.setI2cPortActionFlag(this.dimI2cPort);
        this.dim.writeI2cPortFlagOnlyToController(this.dimI2cPort);
    }

    public String getDeviceName() {
        return "GY-521 Gyro";
    }

    public String getConnectionInfo() {
        return this.dim.getConnectionInfo() + "; I2C port: " + this.dimI2cPort;
    }

    public String status() {
        return String.format("GY-521 Gyro, connected via device %s, port %d", new Object[]{this.dim.getSerialNumber().toString(), Integer.valueOf(this.dimI2cPort)});
    }

    public int getVersion() {
        return 1;
    }

    public void close() {
    }














}
