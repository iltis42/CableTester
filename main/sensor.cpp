
#include "sensor.h"

#include "Cipher.h"
#include "BME280_ESP32_SPI.h"
#include <driver/adc.h>
#include <driver/gpio.h>
#include "mcp3221.h"
#include "mcp4018.h"
#include "ESP32NVS.h"
#include "MP5004DP.h"
#include "MS4525DO.h"
#include "ABPMRR.h"
#include "BMPVario.h"
#include "BTSender.h"
#include "BLESender.h"
#include "Protocols.h"
#include "DS18B20.h"
#include "Setup.h"
#include "ESPAudio.h"
#include "SetupMenu.h"
#include "ESPRotary.h"
#include "AnalogInput.h"
#include "Atmosphere.h"
#include "IpsDisplay.h"
#include "S2F.h"
#include "Version.h"
#include "Polars.h"
#include "Flarm.h"
#include "Blackboard.h"
#include "SetupMenuValFloat.h"

#include <SPI.h>
#include <AdaptUGC.h>
#include <OTA.h>
#include "SetupNG.h"
#include "Switch.h"
#include "AverageVario.h"
#include <list>

#include "MPU.hpp"        // main file, provides the class itself
#include "mpu/math.hpp"   // math helper for dealing with MPU data
#include "mpu/types.hpp"  // MPU data types and definitions
#include "I2Cbus.hpp"
#include "KalmanMPU6050.h"
#include "WifiApp.h"
#include "WifiClient.h"
#include "Serial.h"
#include "LeakTest.h"
#include "Units.h"
#include "Flap.h"
#include "SPL06-007.h"
#include "StraightWind.h"
#include "CircleWind.h"
#include <coredump_to_server.h>
#include "canbus.h"
#include "Router.h"

#include "sdkconfig.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <esp_log.h>
#include <esp32/rom/miniz.h>
#include <esp32-hal-adc.h> // needed for adc pin reset
#include <soc/sens_reg.h> // needed for adc pin reset
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <string>
#include <cstdio>
#include <cstring>
#include "DataMonitor.h"
#include "AdaptUGC.h"
#include "CenterAid.h"

// #include "sound.h"

/*
BMP:
    SCK - This is the SPI Clock pin, its an input to the chip
    SDO - this is the Serial Data Out / Master In Slave Out pin (MISO), for data sent from the BMP183 to your processor
    SDI - this is the Serial Data In / Master Out Slave In pin (MOSI), for data sent from your processor to the BME280
    CS - this is the Chip Select pin, drop it low to start an SPI transaction. Its an input to the chip
 */

#define CS_bme280BA GPIO_NUM_26   // before CS pin 33
#define CS_bme280TE GPIO_NUM_33   // before CS pin 26
#define FREQ_BMP_SPI 13111111/2
#define SPL06_007_BARO 0x77
#define SPL06_007_TE   0x76

#define MGRPS 360

MCP3221 *MCP=0;
DS18B20  ds18b20( GPIO_NUM_23 );  // GPIO_NUM_23 standard, alternative  GPIO_NUM_17

AirspeedSensor *asSensor=0;
StraightWind theWind;

xSemaphoreHandle xMutex=NULL;
xSemaphoreHandle spiMutex=NULL;

S2F Speed2Fly;
Protocols OV( &Speed2Fly );

AnalogInput Battery( (22.0+1.2)/1200, ADC_ATTEN_DB_0, ADC_CHANNEL_7, ADC_UNIT_1 );

TaskHandle_t apid = NULL;
TaskHandle_t bpid = NULL;
TaskHandle_t tpid = NULL;
TaskHandle_t dpid = NULL;

e_wireless_type wireless;

PressureSensor *baroSensor = 0;
PressureSensor *teSensor = 0;

AdaptUGC *MYUCG = 0;  // ( SPI_DC, CS_Display, RESET_Display );
IpsDisplay *display = 0;
CenterAid  *centeraid = 0;

bool topDown = false;

OTA *ota = 0;

ESPRotary Rotary;
SetupMenu  *Menu = 0;
DataMonitor DM;

// Gyro and acceleration sensor
I2C_t& i2c = i2c1;  // i2c0 or i2c1
I2C_t& i2c_0 = i2c0;  // i2c0 or i2c1
MPU_t MPU;         // create an object
mpud::float_axes_t accelG;
mpud::float_axes_t gyroDPS;
mpud::float_axes_t accelG_Prev;
mpud::float_axes_t gyroDPS_Prev;
#define MAXDRIFT 2                // °/s maximum drift that is automatically compensated on ground
#define NUM_GYRO_SAMPLES 3000     // 10 per second -> 5 minutes, so T has been settled after power on
static uint16_t num_gyro_samples = 0;
static int32_t cur_gyro_bias[3];

// Magnetic sensor / compass
Compass *compass = 0;
BTSender btsender;
BLESender blesender;

static float baroP=0; // barometric pressure
static float temperature=15.0;

static float battery=0.0;
static float dynamicP; // Pitot

float slipAngle = 0.0;

// global color variables for adaptable display variant
uint8_t g_col_background=255; // black
uint8_t g_col_highlight=0;
uint8_t g_col_header_r=101+g_col_background/5;
uint8_t g_col_header_g=108+g_col_background/5;
uint8_t g_col_header_b=g_col_highlight;
uint8_t g_col_header_light_r=161-g_col_background/4;
uint8_t g_col_header_light_g=168-g_col_background/3;
uint8_t g_col_header_light_b=g_col_highlight;
uint16_t gear_warning_holdoff = 0;
uint8_t gyro_flash_savings=0;

t_global_flags gflags = { true, false, false, false, false, false, false, false, false, false, false, false, false, false };

int  ccp=60;
float tas = 0;
float cas = 0;
float aTE = 0;
float alt_external;
float altSTD;
float netto = 0;
float as2f = 0;
float s2f_delta = 0;
float polar_sink = 0;

float      stall_alarm_off_kmh=0;
uint16_t   stall_alarm_off_holddown=0;

int count=0;
int flarm_alarm_holdtime=0;
int the_can_mode = CAN_MODE_MASTER;
int active_screen = 0;  // 0 = Vario

float mpu_target_temp=45.0;

AdaptUGC *egl = 0;

#define GYRO_FS (mpud::GYRO_FS_250DPS)

float getTAS() { return tas; };

bool do_factory_reset() {
	return( SetupCommon::factoryReset() );
}

void drawDisplay(void *pvParameters){
	while (1) {
		if( Flarm::bincom ) {
			if( gflags.flarmDownload == false ) {
				gflags.flarmDownload = true;
				display->clear();
				Flarm::drawDownloadInfo();
			}
			// Flarm IGC download is running, display will be blocked, give Flarm
			// download all cpu power.
			vTaskDelay(20/portTICK_PERIOD_MS);
			continue;
		}
		else if( gflags.flarmDownload == true ) {
			gflags.flarmDownload = false;
			display->clear();
		}
		// TickType_t dLastWakeTime = xTaskGetTickCount();
		if( gflags.inSetup != true ) {
			float t=OAT.get();
			if( gflags.validTemperature == false )
				t = DEVICE_DISCONNECTED_C;
			float airspeed = 0;
			if( airspeed_mode.get() == MODE_IAS )
				airspeed = ias.get();
			else if( airspeed_mode.get() == MODE_TAS )
				airspeed = tas;
			else if( airspeed_mode.get() == MODE_CAS )
				airspeed = cas;
			else
				airspeed = ias.get();

			// Stall Warning Screen
			if( stall_warning.get() && gload_mode.get() != GLOAD_ALWAYS_ON ){  // In aerobatics stall warning is contra productive, we concentrate on G-Load Display if permanent enabled
				if( gflags.stall_warning_armed ){
					float acceleration=accelG[0];
					if( acceleration < 0.3 )
						acceleration = 0.3;  // limit acceleration effect to minimum 30% of 1g
					float acc_stall= stall_speed.get() * sqrt( acceleration + ( ballast.get()/100));  // accelerated and ballast(ed) stall speed
					if( ias.get() < acc_stall && ias.get() > acc_stall*0.7 ){
						if( !gflags.stall_warning_active ){
							Audio::alarm( true );
							display->drawWarning( "! STALL !", true );
							gflags.stall_warning_active = true;
						}
					}
					else{
						if( gflags.stall_warning_active ){
							Audio::alarm( false );
							display->clear();
							gflags.stall_warning_active = false;
						}
					}
					if( ias.get() < stall_alarm_off_kmh ){
						stall_alarm_off_holddown++;
						if( stall_alarm_off_holddown > 1200 ){  // ~30 seconds holddown
							gflags.stall_warning_armed = false;
							stall_alarm_off_holddown=0;
						}
					}
					else{
						stall_alarm_off_holddown=0;
					}
				}
				else{
					if( ias.get() > stall_speed.get() ){
						gflags.stall_warning_armed = true;
						stall_alarm_off_holddown=0;
					}
				}
			}
			if( gear_warning.get() ){
				if( !gear_warning_holdoff ){
					int gw = 0;
					if( gear_warning.get() == GW_EXTERNAL ){
						gw = gflags.gear_warn_external;
					}else{
						gw = digitalRead( SetupMenu::getGearWarningIO() );
						if( gear_warning.get() == GW_FLAP_SENSOR_INV || gear_warning.get() == GW_S2_RS232_RX_INV ){
							gw = !gw;
						}
					}
					if( gw ){
						if( ESPRotary::readSwitch() ){   // Acknowledge Warning -> Warning OFF
							gear_warning_holdoff = 25000;  // 5 min
							Audio::alarm( false );
							display->clear();
							gflags.gear_warning_active = false;
							SetupMenu::catchFocus( false );
						}
						else if( !gflags.gear_warning_active && !gflags.stall_warning_active ){
							Audio::alarm( true );
							display->drawWarning( "! GEAR !", false );
							gflags.gear_warning_active = true;
							SetupMenu::catchFocus( true );
						}
					}
					else{
						if( gflags.gear_warning_active ){
							SetupMenu::catchFocus( false );
							Audio::alarm( false );
							display->clear();
							gflags.gear_warning_active = false;
						}
					}
				}
				else{
					gear_warning_holdoff--;
				}
			}

			// Flarm Warning Screen
			if( flarm_warning.get() && !gflags.stall_warning_active && Flarm::alarmLevel() >= flarm_warning.get() ){ // 0 -> Disable
				// ESP_LOGI(FNAME,"Flarm::alarmLevel: %d, flarm_warning.get() %d", Flarm::alarmLevel(), flarm_warning.get() );
				if( !gflags.flarmWarning ) {
					gflags.flarmWarning = true;
					delay(100);
					display->clear();
					flarm_alarm_holdtime = 250;
				}
			}
			else{
				if( gflags.flarmWarning && (flarm_alarm_holdtime == 0) ){
					gflags.flarmWarning = false;
					display->clear();
					Audio::alarm( false );
				}
			}
			if( gflags.flarmWarning )
				Flarm::drawFlarmWarning();
			// G-Load Display
			// ESP_LOGI(FNAME,"Active Screen = %d", active_screen );
			if( (((float)accelG[0] > gload_pos_thresh.get() || (float)accelG[0] < gload_neg_thresh.get()) && gload_mode.get() == GLOAD_DYNAMIC ) ||
					( gload_mode.get() == GLOAD_ALWAYS_ON ) || ((active_screen << SCREEN_GMETER) & 1)  )
			{
				if( !gflags.gLoadDisplay ){
					gflags.gLoadDisplay = true;
				}
			}
			else{
				if( gflags.gLoadDisplay ) {
					gflags.gLoadDisplay = false;
				}
			}
			if( gflags.gLoadDisplay ) {
				display->drawLoadDisplay( (float)accelG[0] );
			}
			// G-Load Alarm when limits reached
			if( gload_mode.get() != GLOAD_OFF  ){
				if( (float)accelG[0] > gload_pos_limit.get() || (float)accelG[0] < gload_neg_limit.get()  ){
					if( !gflags.gload_alarm ) {
						Audio::alarm( true );
						gflags.gload_alarm = true;
					}
				}else
				{
					if( gflags.gload_alarm ) {
						Audio::alarm( false );
						gflags.gload_alarm = false;
					}
				}
			}
			// Vario Screen
			if( !(gflags.stall_warning_active || gflags.gear_warning_active || gflags.flarmWarning || gflags.gLoadDisplay )  ) {
				// ESP_LOGI(FNAME,"TE=%2.3f", te_vario.get() );
				display->drawDisplay( airspeed, te_vario.get(), aTE, polar_sink, altitude.get(), t, battery, s2f_delta, as2f, average_climb.get(), Switch::getCruiseState(), gflags.standard_setting, flap_pos.get() );
			}
			if( screen_centeraid.get() ){
				if( centeraid ){
					centeraid->tick();
				}
			}
		}
		if( flarm_alarm_holdtime )
			flarm_alarm_holdtime--;
		vTaskDelay(20/portTICK_PERIOD_MS);
		if( uxTaskGetStackHighWaterMark( dpid ) < 512  )
			ESP_LOGW(FNAME,"Warning drawDisplay stack low: %d bytes", uxTaskGetStackHighWaterMark( dpid ) );
	}
}

// depending on mode calculate value for Audio and set values accordingly
void doAudio(){
	polar_sink = Speed2Fly.sink( ias.get() );
	float netto = te_vario.get() - polar_sink;
	as2f = Speed2Fly.speed( netto, !Switch::getCruiseState() );
	s2f_delta = s2f_delta + ((as2f - ias.get()) - s2f_delta)* (1/(s2f_delay.get()*10)); // low pass damping moved to the correct place
	// ESP_LOGI( FNAME, "te: %f, polar_sink: %f, netto %f, s2f: %f  delta: %f", aTES2F, polar_sink, netto, as2f, s2f_delta );
	if( vario_mode.get() == VARIO_NETTO || (Switch::getCruiseState() &&  (vario_mode.get() == CRUISE_NETTO)) ){
		if( netto_mode.get() == NETTO_RELATIVE )
			Audio::setValues( te_vario.get() - polar_sink + Speed2Fly.circlingSink( ias.get() ), s2f_delta );
		else if( netto_mode.get() == NETTO_NORMAL )
			Audio::setValues( te_vario.get() - polar_sink, s2f_delta );
	}
	else {
		Audio::setValues( te_vario.get(), s2f_delta );
	}
}

void audioTask(void *pvParameters){
	while (1)
	{
		TickType_t xLastWakeTime = xTaskGetTickCount();
		if( Flarm::bincom ) {
			// Flarm IGC download is running, audio will be blocked, give Flarm
			// download all cpu power.
			vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);
			continue;
		}
		doAudio();
		Router::routeXCV();
		if( uxTaskGetStackHighWaterMark( apid )  < 512 )
			ESP_LOGW(FNAME,"Warning audio task stack low: %d", uxTaskGetStackHighWaterMark( apid ) );
		vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);
	}
}


static void grabMPU()
{
	mpud::raw_axes_t accelRaw;     // holds x, y, z axes as int16
	mpud::raw_axes_t gyroRaw;      // holds x, y, z axes as int16
	esp_err_t err = MPU.acceleration(&accelRaw);  // fetch raw data from the registers
	if( err != ESP_OK )
		ESP_LOGE(FNAME, "accel I2C error, X:%+.2f Y:%+.2f Z:%+.2f", -accelG[2], accelG[1], accelG[0] );
	err |= MPU.rotation(&gyroRaw);       // fetch raw data from the registers
	if( err != ESP_OK )
		ESP_LOGE(FNAME, "gyro I2C error, X:%+.2f Y:%+.2f Z:%+.2f",  gyroDPS.x, gyroDPS.y, gyroDPS.z );

	accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_8G);  // raw data to gravity
	gyroDPS = mpud::gyroDegPerSec(gyroRaw, GYRO_FS);  // raw data to º/s
	// mpud::raw_axes_t gbo = MPU.getGyroOffset();
	// ESP_LOGI(FNAME, "accel X: %+.2f Y:%+.2f Z:%+.2f  gyro X: %+.2f Y:%+.2f Z:%+.2f ABx:%d ABy:%d ABz=%d\n", -accelG[2], accelG[1], accelG[0] ,  gyroDPS.x, gyroDPS.y, gyroDPS.z, gbo.x, gbo.y, gbo.z );
	// if( !(count%60) ){
	//	ESP_LOGI(FNAME, "Gyro X:%+.2f Y:%+.2f Z:%+.2f T=%f\n", gyroDPS.x, gyroDPS.y, gyroDPS.z, MPU.getTemperature());
	// }
	bool goodAccl = true;
	if( abs( accelG.x - accelG_Prev.x ) > 1 || abs( accelG.y - accelG_Prev.y ) > 1 || abs( accelG.z - accelG_Prev.z ) > 1 ) {
		MPU.acceleration(&accelRaw);
		accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_8G);
		if( abs( accelG.x - accelG_Prev.x ) > 1 || abs( accelG.y - accelG_Prev.y ) > 1 || abs( accelG.z - accelG_Prev.z ) > 1 ){
			goodAccl = false;
			ESP_LOGE(FNAME, "accelaration change > 1 g in 0.2 S:  X:%+.2f Y:%+.2f Z:%+.2f", -accelG[2], accelG[1], accelG[0] );
		}
	}
	bool goodGyro = true;
	if( abs( gyroDPS.x - gyroDPS_Prev.x ) > MGRPS || abs( gyroDPS.y - gyroDPS_Prev.y ) > MGRPS || abs( gyroDPS.z - gyroDPS_Prev.z ) > MGRPS ) {
		// ESP_LOGE(FNAME, "gyro sensor out of bounds: X:%+.2f Y:%+.2f Z:%+.2f",  gyroDPS.x, gyroDPS.y, gyroDPS.z );
		// ESP_LOGE(FNAME, "%04x %04x %04x", gyroRaw.x, gyroRaw.y, gyroRaw.z );
		MPU.rotation(&gyroRaw);
		gyroDPS = mpud::gyroDegPerSec(gyroRaw, GYRO_FS );
		if( abs( gyroDPS.x - gyroDPS_Prev.x ) > MGRPS || abs( gyroDPS.y - gyroDPS_Prev.y ) > MGRPS || abs( gyroDPS.z - gyroDPS_Prev.z ) > MGRPS ) {
			goodGyro = false;
			ESP_LOGE(FNAME, "gyro angle >90 deg/s in 0.2 S: X:%+.2f Y:%+.2f Z:%+.2f",  gyroDPS.x, gyroDPS.y, gyroDPS.z );
		}
	}
	if( err == ESP_OK ){
		// check low rotation on all 3 axes = on ground
		if( abs( gyroDPS.x ) < MAXDRIFT && abs( gyroDPS.y ) < MAXDRIFT && abs( gyroDPS.z ) < MAXDRIFT ) {
			if( ias.get() < 25 ){  // check no significant IAS
				num_gyro_samples++;
				for(int i=0; i<3; i++){
					cur_gyro_bias[i] += gyroRaw[i];
				}
				if( num_gyro_samples > NUM_GYRO_SAMPLES ){  // every 5 minute (3000 samples) recalculate offset
					mpud::raw_axes_t gb;
					mpud::raw_axes_t gbo = MPU.getGyroOffset();
					for(int i=0; i<3; i++){
						gb[i]  = gbo[i] -(( (cur_gyro_bias)[i]/(NUM_GYRO_SAMPLES*4)) ); // translate to 1000 DPS
						cur_gyro_bias[i] = 0;
					}
					// ESP_LOGI(FNAME,"New gyro offset X/Y/Z: OLD:%d/%d/%d NEW:%d/%d/%d", gbo.x, gbo.y, gbo.z, gb.x, gb.y, gb.z );
					if( (abs( gbo.x-gb.x ) > 0) || (abs( gbo.y-gb.y ) > 0) || (abs( gbo.z-gb.z ) > 0)  ){  // any delta is directly set in RAM
						ESP_LOGI(FNAME,"Set new gyro offset X/Y/Z: OLD:%d/%d/%d NEW:%d/%d/%d", gbo.x, gbo.y, gbo.z, gb.x, gb.y, gb.z );
						MPU.setGyroOffset( gb );
					}
					// if we have temperature control, we check if control is locked, otherwise we have no idea but anyway takeover better offset
					if( (HAS_MPU_TEMP_CONTROL && (MPU.getSiliconTempStatus() == MPU_T_LOCKED)) || !HAS_MPU_TEMP_CONTROL ){
						if( (abs( gbo.x-gb.x ) > 1 || abs( gbo.y-gb.y ) > 1 || abs( gbo.z-gb.z ) > 1) && gyro_flash_savings<5 ){ // Set only changes > 1 in Flash and only 5 times per boot
							gyro_bias.set( gb );
							ESP_LOGI(FNAME,"Store the new offset also in Flash, store number: %d", gyro_flash_savings );
							gyro_flash_savings++;
						}
					}
					num_gyro_samples = 0;
				}
			}
		}
	}

	if( err == ESP_OK && goodAccl && goodGyro ) {
		IMU::read();
	}
	gyroDPS_Prev = gyroDPS;
	accelG_Prev = accelG;
}

static void lazyNvsCommit()
{
	uint16_t dummy;
	if ( xQueueReceive(SetupCommon::commitSema, &dummy, 0) ) {
		SetupCommon::commitNow();
	}
}

static void toyFeed()
{
	xSemaphoreTake(xMutex,portMAX_DELAY );
	// reduce also messages from 10 per second to 5 per second to reduce load in XCSoar
	// maybe just 1 or 2 per second
	static char lb[150];

	if( ahrs_rpyl_dataset.get() ){
		OV.sendNMEA( P_AHRS_RPYL, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature,
				-accelG[2], accelG[1],accelG[0], gyroDPS.x, gyroDPS.y, gyroDPS.z );
		OV.sendNMEA( P_AHRS_APENV1, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature,
				-accelG[2], accelG[1],accelG[0], gyroDPS.x, gyroDPS.y, gyroDPS.z );
	}
	if( nmea_protocol.get() == BORGELT ) {
		OV.sendNMEA( P_BORGELT, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altSTD, gflags.validTemperature  );
		OV.sendNMEA( P_GENERIC, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altSTD, gflags.validTemperature  );
	}
	else if( nmea_protocol.get() == OPENVARIO ){
		OV.sendNMEA( P_OPENVARIO, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature  );
	}
	else if( nmea_protocol.get() == CAMBRIDGE ){
		OV.sendNMEA( P_CAMBRIDGE, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature  );
	}
	else if( nmea_protocol.get() == XCVARIO ) {
		OV.sendNMEA( P_XCVARIO, lb, baroP, dynamicP, te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), Switch::getCruiseState(), altitude.get(), gflags.validTemperature,
				-accelG[2], accelG[1],accelG[0], gyroDPS.x, gyroDPS.y, gyroDPS.z );
	}
	else if( nmea_protocol.get() == NMEA_OFF ) {
		;
	}
	else
		ESP_LOGE(FNAME,"Protocol %d not supported error", nmea_protocol.get() );
	xSemaphoreGive(xMutex);
}


void clientLoop(void *pvParameters)
{
	int ccount = 0;
	gflags.validTemperature = true;
	while (true)
	{
		TickType_t xLastWakeTime = xTaskGetTickCount();
		ccount++;
		aTE += (te_vario.get() - aTE)* (1/(10*vario_av_delay.get()));
		if( gflags.haveMPU ) {
			grabMPU();
		}
		if( !(ccount%5) )
		{
			double tmpalt = altitude.get(); // get pressure from altitude
			if( (fl_auto_transition.get() == 1) && ((int)( Units::meters2FL( altitude.get() )) + (int)(gflags.standard_setting) > transition_alt.get() ) ) {
				ESP_LOGI(FNAME,"Above transition altitude");
				baroP = baroSensor->calcPressure(1013.25, tmpalt); // above transition altitude
			}
			else {
				baroP = baroSensor->calcPressure( QNH.get(), tmpalt);
			}
			dynamicP = Atmosphere::kmh2pascal(ias.get());
			tas = Atmosphere::TAS2( ias.get(), altitude.get(), OAT.get() );
			if( airspeed_mode.get() == MODE_CAS )
				cas = Atmosphere::CAS( dynamicP );
			if( gflags.haveMPU && HAS_MPU_TEMP_CONTROL ){
				MPU.temp_control( ccount );
			}
			if( accelG[0] > gload_pos_max.get() ){
				gload_pos_max.set( (float)accelG[0] );
			}else if( accelG[0] < gload_neg_max.get() ){
				gload_neg_max.set(  (float)accelG[0] );
			}
			toyFeed();
			Router::routeXCV();
			if( true && !(ccount%5) ) { // todo need a mag_hdm.valid() flag
				if( compass_nmea_hdm.get() ) {
					xSemaphoreTake( xMutex, portMAX_DELAY );
					OV.sendNmeaHDM( mag_hdm.get() );
					xSemaphoreGive( xMutex );
				}

				if( compass_nmea_hdt.get() ) {
					xSemaphoreTake( xMutex, portMAX_DELAY );
					OV.sendNmeaHDT( mag_hdt.get() );
					xSemaphoreGive( xMutex );
				}
			}
			lazyNvsCommit();
			esp_task_wdt_reset();
			if( uxTaskGetStackHighWaterMark( bpid ) < 512 )
				ESP_LOGW(FNAME,"Warning client task stack low: %d bytes", uxTaskGetStackHighWaterMark( bpid ) );
		}
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
	}
}

void readSensors(void *pvParameters){
	int client_sync_dataIdx = 0;
	while (1)
	{
		count++;
		TickType_t xLastWakeTime = xTaskGetTickCount();
		if( gflags.haveMPU  )  // 3th Generation HW, MPU6050 avail and feature enabled
		{
			grabMPU();
		}
		bool ok=false;
		float p = 0;
		if( asSensor )
			p = asSensor->readPascal(60, ok);
		if( ok )
			dynamicP = p;
		float iasraw = Atmosphere::pascal2kmh( dynamicP );
		// ESP_LOGI("FNAME","P: %f  IAS:%f", dynamicP, iasraw );
		float T=OAT.get();
		if( !gflags.validTemperature ) {
			T= 15 - ( (altitude.get()/100) * 0.65 );
			// ESP_LOGW(FNAME,"T invalid, using 15 deg");
		}
		float tasraw = 0;
		if( baroP != 0 )
			tasraw =  Atmosphere::TAS( iasraw , baroP, T);  // True airspeed in km/h

		if( airspeed_mode.get() == MODE_CAS ){
			float casraw=Atmosphere::CAS( dynamicP );
			cas += (casraw-cas)*0.25;       // low pass filter
			// ESP_LOGI(FNAME,"IAS=%f, TAS=%f CAS=%f baro=%f", iasraw, tasraw, cas, baroP );
		}
		static float new_ias = 0;
		new_ias = ias.get() + (iasraw - ias.get())*0.25;
		if( (int( ias.get()+0.5 ) != int( new_ias+0.5 ) ) || !(count%20) ){
			ias.set( new_ias );  // low pass filter
		}
		if( airspeed_max.get() < ias.get() ){
			airspeed_max.set( ias.get() );
		}
		// ESP_LOGI("FNAME","P: %f  IAS:%f IASF: %d", dynamicP, iasraw, ias );
		if( !compass || !(compass->externalData()) ){
			tas += (tasraw-tas)*0.25;       // low pass filter
		}
		// ESP_LOGI(FNAME,"IAS=%f, T=%f, TAS=%f baroP=%f", ias, T, tas, baroP );

		// Slip angle estimation
		float as = tas/3.6;                  // tas in m/s
		const float K = 4000 * 180/M_PI;      // airplane constant and Ay correction factor
		if( tas > 25.0 ){
			slipAngle += ((accelG[1]*K / (as*as)) - slipAngle)*0.09;   // with atan(x) = x for small x
			// ESP_LOGI(FNAME,"AS: %f m/s, CURSL: %f°, SLIP: %f", as, -accelG[1]*K / (as*as), slipAngle );
		}
		xSemaphoreTake(xMutex,portMAX_DELAY );

		float te = bmpVario.readTE( tasraw );
		if( (int( te_vario.get()*20 +0.5 ) != int( te*20 +0.5)) || !(count%10) ){  // a bit more fine granular updates than 0.1 m/s as of sound
			te_vario.set( te );  // max 10x per second
		}
		xSemaphoreGive(xMutex);
		// ESP_LOGI(FNAME,"count %d ccp %d", count, ccp );
		if( !(count % ccp) ) {
			AverageVario::recalcAvgClimb();
		}
		if (FLAP) { FLAP->progress(); }
		xSemaphoreTake(xMutex,portMAX_DELAY );
		baroP = baroSensor->readPressure(ok);   // 10x per second
		xSemaphoreGive(xMutex);
		// ESP_LOGI(FNAME,"Baro Pressure: %4.3f", baroP );
		float altSTD = 0;
		if( Flarm::validExtAlt() && alt_select.get() == AS_EXTERNAL )
			altSTD = alt_external;
		else
			altSTD = baroSensor->calcAVGAltitudeSTD( baroP );
		float new_alt = 0;
		if( alt_select.get() == AS_TE_SENSOR ) // TE
			new_alt = bmpVario.readAVGalt();
		else if( alt_select.get() == AS_BARO_SENSOR  || alt_select.get() == AS_EXTERNAL ){ // Baro or external
			if(  alt_unit.get() == ALT_UNIT_FL ) { // FL, always standard
				new_alt = altSTD;
				gflags.standard_setting = true;
				// ESP_LOGI(FNAME,"au: %d", alt_unit.get() );
			}else if( (fl_auto_transition.get() == 1) && ((int)Units::meters2FL( altSTD ) + (int)(gflags.standard_setting) > transition_alt.get() ) ) { // above transition altitude
				new_alt = altSTD;
				gflags.standard_setting = true;
				ESP_LOGI(FNAME,"auto:%d alts:%f ss:%d ta:%f", fl_auto_transition.get(), altSTD, gflags.standard_setting, transition_alt.get() );
			}
			else {
				if( Flarm::validExtAlt() && alt_select.get() == AS_EXTERNAL )
					new_alt = altSTD + ( QNH.get()- 1013.25)*8.2296;  // correct altitude according to ISA model = 27ft / hPa
				else
					new_alt = baroSensor->calcAVGAltitude( QNH.get(), baroP );
				gflags.standard_setting = false;
				// ESP_LOGI(FNAME,"QNH %f baro: %f alt: %f SS:%d", QNH.get(), baroP, alt, gflags.standard_setting  );
			}
		}
		if( (int( new_alt +0.5 ) != int( altitude.get() +0.5 )) || !(count%20) ){
			// ESP_LOGI(FNAME,"New Altitude: %.1f", new_alt );
			altitude.set( new_alt );
		}

		aTE = bmpVario.readAVGTE();
		doAudio();

		if( !Flarm::bincom && ((count % 2) == 0 ) ){
			toyFeed();
			vTaskDelay(2/portTICK_PERIOD_MS);
		}
		Router::routeXCV();
		// ESP_LOGI(FNAME,"Compass, have sensor=%d  hdm=%d ena=%d", compass->haveSensor(),  compass_nmea_hdt.get(),  compass_enable.get() );
		if( compass ){
			if( !Flarm::bincom && ! compass->calibrationIsRunning() ) {
				// Trigger heading reading and low pass filtering. That job must be
				// done periodically.
				bool ok;
				float heading = compass->getGyroHeading( &ok );
				if(ok){
					if( (int)heading != (int)mag_hdm.get() && !(count%10) ){
						mag_hdm.set( heading );
					}
					if( !(count%5) && compass_nmea_hdm.get() == true ) {
						xSemaphoreTake( xMutex, portMAX_DELAY );
						OV.sendNmeaHDM( heading );
						xSemaphoreGive( xMutex );
					}
				}
				else{
					if( mag_hdm.get() != -1 )
						mag_hdm.set( -1 );
				}
				float theading = compass->filteredTrueHeading( &ok );
				if(ok){
					if( (int)theading != (int)mag_hdt.get() && !(count%10) ){
						mag_hdt.set( theading );
					}
					if( !(count%5) && ( compass_nmea_hdt.get() == true )  ) {
						xSemaphoreTake( xMutex, portMAX_DELAY );
						OV.sendNmeaHDT( theading );
						xSemaphoreGive( xMutex );
					}
				}
				else{
					if( mag_hdt.get() != -1 )
						mag_hdt.set( -1 );
				}
			}
		}
		if( accelG[0] > gload_pos_max.get() ){
			gload_pos_max.set( (float)accelG[0] );
		}else if( accelG[0] < gload_neg_max.get() ){
			gload_neg_max.set(  (float)accelG[0] );
		}

		// Check on new clients connecting
		if ( CAN && CAN->GotNewClient() ) { // todo use also for Wifi client?!
			while( client_sync_dataIdx < SetupCommon::numEntries() ) {
				if ( SetupCommon::syncEntry(client_sync_dataIdx++) ) {
					break; // Hit entry to actually sync and send data
				}
			}
			if ( client_sync_dataIdx >= SetupCommon::numEntries() ) {
				// Synch complete
				client_sync_dataIdx = 0;
				CAN->ResetNewClient();
			}
		}
		lazyNvsCommit();
		if( gflags.haveMPU && HAS_MPU_TEMP_CONTROL ){
			// ESP_LOGI(FNAME,"MPU temp control; T=%.2f", MPU.getTemperature() );
			MPU.temp_control( count );
		}
		esp_task_wdt_reset();
		if( uxTaskGetStackHighWaterMark( bpid ) < 512 )
			ESP_LOGW(FNAME,"Warning sensor task stack low: %d bytes", uxTaskGetStackHighWaterMark( bpid ) );
		vTaskDelayUntil(&xLastWakeTime, 100/portTICK_PERIOD_MS);
	}
}

static int ttick = 0;
static float temp_prev = -3000;

void readTemp(void *pvParameters){

	while (1) {
		TickType_t xLastWakeTime = xTaskGetTickCount();
		float t=15.0;
		battery = Battery.get();
		// ESP_LOGI(FNAME,"Battery=%f V", battery );
		if( !SetupCommon::isClient() ) {  // client Vario will get Temperature info from main Vario
			t = ds18b20.getTemp();
			// ESP_LOGI(FNAME,"Temp %f", t );
			if( t ==  DEVICE_DISCONNECTED_C ) {
				if( gflags.validTemperature == true ) {
					ESP_LOGI(FNAME,"Temperatur Sensor disconnected");
					gflags.validTemperature = false;
				}
			}
			else
			{
				if( gflags.validTemperature == false ) {
					ESP_LOGI(FNAME,"Temperatur Sensor connected");
					gflags.validTemperature = true;
				}
				// ESP_LOGI(FNAME,"temperature=%2.1f", temperature );
				temperature +=  (t - temperature) * 0.3; // A bit low pass as strategy against toggling
				temperature = std::round(temperature*10)/10;
				if( temperature != temp_prev ){
					OAT.set( temperature );
					ESP_LOGI(FNAME,"NEW temperature=%2.1f, prev T=%2.1f", temperature, temp_prev );
					temp_prev = temperature;
				}
			}
			// ESP_LOGV(FNAME,"T=%f", temperature );
			Flarm::tick();
			if( compass )
				compass->tick();
		}else{
			if( (OAT.get() > -55.0) && (OAT.get() < 85.0) )
				gflags.validTemperature = true;
		}
		theWind.tick();
		CircleWind::tick();
		Flarm::progress();
		vTaskDelayUntil(&xLastWakeTime, 1000/portTICK_PERIOD_MS);
		esp_task_wdt_reset();
		if( (ttick++ % 50) == 0) {
			ESP_LOGI(FNAME,"Free Heap: %d bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT) );
			if( uxTaskGetStackHighWaterMark( tpid ) < 256 )
				ESP_LOGW(FNAME,"Warning temperature task stack low: %d bytes", uxTaskGetStackHighWaterMark( tpid ) );
			if( heap_caps_get_free_size(MALLOC_CAP_8BIT) < 20000 )
				ESP_LOGW(FNAME,"Warning heap_caps_get_free_size getting low: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
		}
	}
}

static esp_err_t _coredump_to_server_begin_cb(void * priv)
{
	ets_printf("================= CORE DUMP START =================\r\n");
	return ESP_OK;
}

static esp_err_t _coredump_to_server_end_cb(void * priv)
{
	ets_printf("================= CORE DUMP END ===================\r\n");
	return ESP_OK;
}

static esp_err_t _coredump_to_server_write_cb(void * priv, char const * const str)
{
	ets_printf("%s\r\n", str);
	return ESP_OK;
}

void register_coredump() {
	coredump_to_server_config_t coredump_cfg = {
			.start = _coredump_to_server_begin_cb,
			.end = _coredump_to_server_end_cb,
			.write = _coredump_to_server_write_cb,
			.priv = NULL,
	};
	if( coredump_to_server(&coredump_cfg) != ESP_OK ){  // Dump to console and do not clear (will done after fetched from Webserver)
		ESP_LOGI( FNAME, "+++ All green, no coredump found in FLASH +++");
	}
}


std::list<int> allpins = { 2, 4, 12, 16, 17, 18, 19, 21, 22, 25, 26, 33, 34, 35, 36, 39 };

/*
gpio_set_direction(_sw, GPIO_MODE_INPUT);
gpio_set_pull_mode(_sw, GPIO_PULLDOWN_ONLY);
gpio_set_pull_mode(_sw, GPIO_PULLUP_ONLY);
int level = gpio_get_level(_sw );
*/

bool checkLow( std::list<int> pins ){
   for( std::list<int>::iterator iter= pins.begin(); iter != pins.end(); iter++ ){
	 if( gpio_get_level( (gpio_num_t)*iter ) == 1 ){
		 ESP_LOGI( FNAME, "test low failed at I/O %d", *iter );
		 return false;
	 }
   }
   return true;
}

bool checkHighList( std::list<int> pins ){
   for( std::list<int>::iterator iter= pins.begin(); iter != pins.end(); iter++ ){
	 int checkPin = *iter;
	 if( gpio_get_level( (gpio_num_t)checkPin ) == 0 ){
		 ESP_LOGI( FNAME, "check high %d failed, read low", checkPin );
		 return false;
	 }
   }
   return true;
}

bool checkHighPin( int pin ){
	if( gpio_get_level( (gpio_num_t)pin ) == 0 ){
		ESP_LOGI( FNAME, "check high %d failed, read low", pin );
		return false;
	}
	return true;
}

void setPullUp( std::list<int> pins ){
	for( std::list<int>::iterator iter=pins.begin(); iter != pins.end(); iter++ ){
			gpio_set_direction( (gpio_num_t)*iter, GPIO_MODE_INPUT);
			gpio_set_pull_mode( (gpio_num_t)*iter, GPIO_PULLUP_ONLY);
		}
}

void allAsInput(){
	for( std::list<int>::iterator iter= allpins.begin(); iter != allpins.end(); iter++ ){
			gpio_set_direction( (gpio_num_t)*iter, GPIO_MODE_INPUT);
			gpio_set_pull_mode( (gpio_num_t)*iter, GPIO_PULLDOWN_ONLY);
		}
}

bool test_connection( const char *name, int output, std::list<int> input ){
	ESP_LOGI( FNAME, "test_connection: %s, output: %d", name, output );
	bool ret=true;
	allAsInput();
	gpio_set_direction( (gpio_num_t)output, GPIO_MODE_OUTPUT );
	gpio_set_level( (gpio_num_t)output, 1 );
	delay(10);
	for( std::list<int>::iterator iter=input.begin(); iter !=input.end(); iter++ ){
		int in=*iter;
		if( !checkHighPin( in  ) ){
			ret = false;
			ESP_LOGI( FNAME, "test pin: %d high failed", in );
		}
	}
	std::list<int> allButThisTest=allpins;
	allButThisTest.remove( output );
	for( std::list<int>::iterator iter=input.begin(); iter !=input.end(); iter++ ){
		int input = *iter;
		allButThisTest.remove( input );
	}
	if( !checkLow( allButThisTest ) ){
		ret=false;
		ESP_LOGI( FNAME, "test all pins low failed");
	}
	return ret;
}


// Sensor board init method. Herein all functions that make the XCVario are launched and tested.
void system_startup(void *args){
	accelG[0] = 1;  // earth gravity default = 1 g
	accelG[1] = 0;
	accelG[2] = 0;
	gyroDPS.x = 0;
	gyroDPS.y = 0;
	gyroDPS.z = 0;
	cur_gyro_bias[0] = 0;
	cur_gyro_bias[1] = 0;
	cur_gyro_bias[2] = 0;

	bool selftestPassed=true;
	int line = 1;
	ESP_LOGI( FNAME, "Now setup I2C bus IO 21/22");
	i2c.begin(GPIO_NUM_21, GPIO_NUM_22, 100000 );
	theWind.begin();

	MCP = new MCP3221();
	MCP->setBus( &i2c );
	gpio_set_drive_capability(GPIO_NUM_23, GPIO_DRIVE_CAP_1);

	esp_wifi_set_mode(WIFI_MODE_NULL);
	spiMutex = xSemaphoreCreateMutex();
	Menu = new SetupMenu();
	// esp_log_level_set("*", ESP_LOG_INFO);
	ESP_LOGI( FNAME, "Log level set globally to INFO %d; Max Prio: %d Wifi: %d",  ESP_LOG_INFO, configMAX_PRIORITIES, ESP_TASKD_EVENT_PRIO-5 );
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	ESP_LOGI( FNAME,"This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
					(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
	ESP_LOGI( FNAME,"Silicon revision %d, ", chip_info.revision);
	ESP_LOGI( FNAME,"%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
	ESP_LOGI(FNAME, "QNH.get() %.1f hPa", QNH.get() );
	NVS.begin();
	register_coredump();

	wireless = (e_wireless_type)(wireless_type.get()); // we cannot change this on the fly, so get that on boot
	AverageVario::begin();
	stall_alarm_off_kmh = stall_speed.get()/3;

	xMutex=xSemaphoreCreateMutex();
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	ccp = (int)(core_climb_period.get()*10);
	SPI.begin( SPI_SCLK, SPI_MISO, SPI_MOSI, CS_bme280BA );
	xSemaphoreGive(spiMutex);

	egl = new AdaptUGC();
	egl->begin();
	xSemaphoreTake(spiMutex,portMAX_DELAY );
	ESP_LOGI( FNAME, "setColor" );
	egl->setColor( 0, 255, 0 );
	ESP_LOGI( FNAME, "drawLine" );
	egl->drawLine( 20,20, 20,80 );
	ESP_LOGI( FNAME, "finish Draw" );
	xSemaphoreGive(spiMutex);

	MYUCG = egl; // new AdaptUGC( SPI_DC, CS_Display, RESET_Display );
	display = new IpsDisplay( MYUCG );
	Flarm::setDisplay( MYUCG );
	DM.begin( MYUCG );
	display->begin();
	display->bootDisplay();

	// int valid;
	String logged_tests;
	logged_tests += "\n\n\n";
	char id[16] = { 0 };
	strcpy( id, custom_wireless_id.get().id );
	ESP_LOGI(FNAME,"Custom Wirelss-ID from Flash: %s len: %d", id, strlen(id) );
	if( strlen( id ) == 0 ){
		custom_wireless_id.set( SetupCommon::getDefaultID() ); // Default ID created from MAC address CRC
		ESP_LOGI(FNAME,"Empty ID: Initialize empty Wirelss-ID: %s", custom_wireless_id.get().id );
	}
	ESP_LOGI(FNAME,"Custom Wirelss-ID: %s", custom_wireless_id.get().id );

	String wireless_id;
	if( wireless == WL_BLUETOOTH ) {
		wireless_id="BT ID: ";
		btsender.begin();
	}
	else if( wireless == WL_BLUETOOTH_LE ){
		blesender.begin();
	}
	else
		wireless_id="WLAN SID: ";
	wireless_id += SetupCommon::getID();

	Cipher::begin();

	ESP_LOGI(FNAME,"Now start T sensor test");
	// Temp Sensor test
	if( !SetupCommon::isClient()  ) {
		ESP_LOGI(FNAME,"Now start T sensor test");
		ds18b20.begin();
		temperature = ds18b20.getTemp();
		ESP_LOGI(FNAME,"End T sensor test");
		if( temperature == DEVICE_DISCONNECTED_C ) {
			ESP_LOGE(FNAME,"Error: Self test Temperatur Sensor failed; returned T=%2.2f", temperature );
			display->writeText( line++, "Temp Sensor: NOT FOUND");
			gflags.validTemperature = false;
			logged_tests += "External Temperature Sensor: NOT FOUND\n";
		}else
		{
			ESP_LOGI(FNAME,"Self test Temperatur Sensor PASSED; returned T=%2.2f", temperature );
			display->writeText( line++, "Temp Sensor: OK");
			gflags.validTemperature = true;
			logged_tests += "External Temperature Sensor:PASSED\n";

		}
	}
	esp_task_wdt_reset();

	if( !checkHighPin( 23  ) ){
		display->writeText( line++, "TSEN: +3.3V S1-6: FAILED");
		selftestPassed = false;
	}else
		display->writeText( line++, "TSEN: +3.3V S1-6: OK");

	if( !test_connection( "+12V", 4, { 39, 36, 26 } ) ){
		display->writeText( line++, "+12V: S1-1;S2-7,8: FAILED");
		selftestPassed = false;
	}else
		display->writeText( line++, "+12V: S1-1;S2-7,8: OK");

	setPullUp( { 12, 25, 33, 35 } );    // Its all grounded via S1 Pin 7, check if all is low
	if( !checkLow( { 12, 25  } ) ){
		display->writeText( line++, "GND : S1-8,7;S2-1: FAILED");
		selftestPassed = false;
	}else
		display->writeText( line++, "GND : S1-1;S2-7,8: OK");

	if( !checkLow( { 33 } ) ){
		display->writeText( line++, "PWR : PWR-GND: FAILED");
		selftestPassed = false;
	}else
		display->writeText( line++, "PWR : PWR-GND: OK");

	if( !checkLow( { 35 } ) ){
		display->writeText( line++, "S2F-SW:   GND: FAILED");
		selftestPassed = false;
	}else
		display->writeText( line++, "S2F-SW:   GND: OK");

	if( !test_connection( "S2F-SW", 17, { 34 } ) ){
		display->writeText( line++, "S2F-SW:  S1-4: FAILED");
		selftestPassed = false;
	}else
		display->writeText( line++, "S2F-SW:  S1-4: OK");


	if( !test_connection( "FLARM-TX", 21, { 19 } ) ){
		display->writeText( line++, "FLTX: S1-4/S2-2: FAILED");
		selftestPassed = false;
	}else
		display->writeText( line++, "FLTX: S1-4/S2-2: OK");

	if( !test_connection( "FLARM-RX", 22, { 18 } ) ){
		display->writeText( line++, "FLRX: S1-3/S2-3: FAILED");
		selftestPassed = false;
	}else
		display->writeText( line++, "FLRX: S1-3/S2-3: OK");



	Version myVersion;
	ESP_LOGI(FNAME,"Program Version %s", myVersion.version() );
	ESP_LOGI(FNAME,"%s", logged_tests.c_str());
	if( !selftestPassed )
	{
		ESP_LOGI(FNAME,"\n\n\nTEST failed, see above LOG for Problems\n\n\n");
		display->writeText( line++, "Cable FAILED", COLOR_RED );
		if( !Rotary.readSwitch() )
			sleep(4);
	}
	else{
		ESP_LOGI(FNAME,"\n\n\n*****  Selftest PASSED  ********\n\n\n");
		display->getDisplay()->setColor( COLOR_GREEN );
		display->writeText( line++, "Cable PASSED", COLOR_GREEN );
	}
	delay( 100000 );
}

extern "C" void  app_main(void)
{
	// Init timer infrastructure
	esp_timer_init();
	Router::begin();
	Audio::boot();
	ESP_LOGI(FNAME,"app_main" );
	ESP_LOGI(FNAME,"Now init all Setup elements");
	bool setupPresent;
	SetupCommon::initSetup( setupPresent );
	Cipher::begin();
	if( !setupPresent ){
		if( Cipher::init() )
			attitude_indicator.set(1);
	}
	else
		ESP_LOGI(FNAME,"Setup already present");
	esp_log_level_set("*", ESP_LOG_INFO);

	system_startup( 0 );
	vTaskDelete( NULL );
}
