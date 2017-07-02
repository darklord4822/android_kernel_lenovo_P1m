//s_add new sensor driver here
//export funtions
/*OV*/
UINT32 OV8865_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 OV8865_MIPI_RAW_Sunny_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
/*HI*/
UINT32 HI551_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 HI551AVC_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
//! Add Sensor Init function here
//! Note:
//! 1. Add by the resolution from ""large to small"", due to large sensor
//!    will be possible to be main sensor.
//!    This can avoid I2C error during searching sensor.
//! 2. This file should be the same as mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR+1] =
{
//8M
#if defined(OV8865_MIPI_RAW)
    {OV8865_SENSOR_ID, SENSOR_DRVNAME_OV8865_MIPI_RAW,OV8865_MIPI_RAW_SensorInit},
#endif
#if defined(OV8865_MIPI_RAW_SUNNY)
    {OV8865_SUNNY_SENSOR_ID, SENSOR_DRVNAME_OV8865_MIPI_RAW_SUNNY,OV8865_MIPI_RAW_Sunny_SensorInit},
#endif
//5M
#if defined(HI551_MIPI_RAW)
    {HI551_SENSOR_ID, SENSOR_DRVNAME_HI551_MIPI_RAW,HI551_MIPI_RAW_SensorInit},
#endif
#if defined(HI551AVC_MIPI_RAW)
    {HI551AVC_SENSOR_ID, SENSOR_DRVNAME_HI551AVC_MIPI_RAW,HI551AVC_MIPI_RAW_SensorInit},
#endif

/*  ADD sensor driver before this line */
    {0,{0},NULL}, //end of list
};
//e_add new sensor driver here

