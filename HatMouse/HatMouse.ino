// Jacek Fedorynski <jfedor@jfedor.org>
// http://www.jfedor.org/

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <bluefruit.h>

// adjust these to your liking
#define SENSITIVITY 768
#define VERTICAL_SENSITIVITY_MULTIPLIER 2
// 0 <= SMOOTHING_RATIO < 1, 0 = no smoothing
#define SMOOTHING_RATIO 0.8

// PIN_BUTTON1 is the "user switch" on the Feather nRF52840 Sense
#define RECENTER_BUTTON PIN_BUTTON1

#define MOUSE_REPORT_ID 1

Adafruit_LSM6DS33 lsm6ds;
Adafruit_LIS3MDL lis3mdl;
Adafruit_Sensor_Calibration_SDFat cal;

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

Adafruit_NXPSensorFusion filter;
//Adafruit_Madgwick filter;
//Adafruit_Mahony filter;

BLEDis bledis;
BLEHidGeneric blehid = BLEHidGeneric(1);

// screen center position
float yaw0 = 0.0;
float pitch0 = 0.0;

int32_t x;
int32_t y;

uint32_t timestamp;

// we use our own descriptor because we want absolute mouse positioning
uint8_t const hid_report_descriptor[] = {
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
  HID_USAGE(HID_USAGE_DESKTOP_MOUSE),
  HID_COLLECTION(HID_COLLECTION_APPLICATION),
  HID_REPORT_ID(1)
  HID_USAGE(HID_USAGE_DESKTOP_POINTER),
  HID_COLLECTION(HID_COLLECTION_PHYSICAL),
  HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),
  HID_USAGE_MIN(1),
  HID_USAGE_MAX(3),
  HID_LOGICAL_MIN(0),
  HID_LOGICAL_MAX(1),
  /* buttons */
  HID_REPORT_COUNT(3),
  HID_REPORT_SIZE(1),
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
  /* padding */
  HID_REPORT_COUNT(1),
  HID_REPORT_SIZE(5),
  HID_INPUT(HID_CONSTANT),
  HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
  /* X, Y position [0, 32767] */
  HID_USAGE(HID_USAGE_DESKTOP_X),
  HID_USAGE(HID_USAGE_DESKTOP_Y),
  HID_LOGICAL_MIN_N(0x0000, 2),
  HID_LOGICAL_MAX_N(0x7fff, 2),
  HID_REPORT_COUNT(2),
  HID_REPORT_SIZE(16),
  HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
  HID_COLLECTION_END,
  HID_COLLECTION_END
};

void setup() {
  pinMode(RECENTER_BUTTON, INPUT_PULLUP);

  // assuming we followed the magnetometer calibration procedure at
  // https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions
  cal.begin();
  cal.loadCalibration();

  lsm6ds.begin_I2C();
  lis3mdl.begin_I2C();
  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  filter.begin(66); // Hz

  Bluefruit.configPrphBandwidth(BANDWIDTH_HIGH);
  Bluefruit.begin();
  Bluefruit.setName("Hat Mouse");

  // Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Feather Sense");
  bledis.begin();

  blehid.setReportMap(hid_report_descriptor, sizeof(hid_report_descriptor));
  uint16_t input_len [] = { 5 };
  blehid.setReportLen(input_len, NULL, NULL);
  blehid.begin();

  Bluefruit.Periph.setConnInterval(12, 12); // 12*1.25ms=15ms

  startAdvertising();

  Wire.setClock(400000);
}

void startAdvertising(void)
{
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void mousePosition(int16_t x, int16_t y) {
  uint8_t report[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
  report[0] = 0;
  report[1] = x & 0xff;
  report[2] = (x >> 8) & 0xff;
  report[3] = y & 0xff;
  report[4] = (y >> 8) & 0xff;
  blehid.inputReport(MOUSE_REPORT_ID, report, sizeof(report));
}

void loop() {
  // we target 66 Hz
  if ((millis() - timestamp) < (15)) {
    return;
  }
  timestamp = millis();

  sensors_event_t accel, gyro, mag;

  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  float gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  float gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  float gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  filter.update(gx, gy, gz,
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  float yaw = filter.getYaw();
  float pitch = filter.getPitch();

  if (digitalRead(RECENTER_BUTTON) == LOW) {
    yaw0 = yaw;
    pitch0 = pitch;
  }

  x = SMOOTHING_RATIO * x + (1 - SMOOTHING_RATIO) * (16384 + -(yaw - yaw0) * SENSITIVITY);
  x = max(0, min(32767, x));
  y = SMOOTHING_RATIO * y + (1 - SMOOTHING_RATIO) * (16384 + -(pitch - pitch0) * SENSITIVITY * VERTICAL_SENSITIVITY_MULTIPLIER);
  y = max(0, min(32767, y));
  mousePosition(x, y);
}
