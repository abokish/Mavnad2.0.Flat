# OTA Health Check System

## Overview

The OTA Health Check System is a robust firmware validation mechanism that ensures OTA updates are reliable and safe. Instead of immediately marking firmware as valid after boot, it requires the firmware to prove its stability through a series of health checks before being permanently installed.

## How It Works

### 1. **Delayed Validation**
- After an OTA update, firmware is NOT immediately marked as valid
- The system enters a "validation period" where it monitors the firmware's behavior
- Only after proving stability is the firmware permanently installed

### 2. **Health Check Requirements**
- **15 successful loop iterations** must complete
- **5-minute maximum validation time** (configurable)
- **System health checks** (WiFi, sensors, loop execution time)
- **Endless loop detection** (max 10 seconds per loop)

### 3. **Automatic Rollback**
If any of these conditions are met during validation:
- System becomes unhealthy (WiFi down, sensors not responding)
- Validation timeout (5 minutes)
- Endless loop detected (loop takes >10 seconds)
- Watchdog timeout

The system automatically rolls back to the previous working firmware version.

## Configuration Constants

```cpp
const unsigned int REQUIRED_HEALTH_CHECKS = 15;        // 15 successful loops required
const unsigned long MAX_VALIDATION_TIME = 5 * 60 * 1000;  // 5 minutes timeout
const unsigned long HEARTBEAT_INTERVAL = 30 * 1000;   // 30 seconds heartbeat
const unsigned long MAX_LOOP_TIME = 10 * 1000;        // 10 seconds max loop time
```

## System States

### **PENDING_VALIDATION**
- Firmware downloaded and installed
- System booted with new firmware
- Health checks begin

### **VALIDATING**
- Health check counter incrementing
- Heartbeat monitoring active
- System health being verified

### **VALIDATED**
- All health checks passed
- Firmware marked as permanently valid
- Rollback protection enabled

### **ROLLBACK**
- Health check failed
- System rolling back to previous firmware
- Automatic reboot initiated

## Health Check Process

### **Loop 1-15:**
1. Increment health check counter
2. Verify system health (WiFi, sensors)
3. Check loop execution time
4. Send heartbeat to ThingsBoard
5. Save progress

### **Validation Success:**
- Mark firmware as valid
- Cancel rollback protection
- Send confirmation to ThingsBoard
- Continue normal operation

### **Validation Failure:**
- Trigger automatic rollback
- Send failure notification
- Reboot with previous firmware

## Monitoring & Telemetry

### **ThingsBoard Telemetry:**
- `fw_state`: Current firmware state
- `fw_version_validating`: Version being validated
- `fw_validation_progress`: Health check progress
- `fw_validation_time_elapsed`: Time since validation started
- `OTA_Status`: Overall OTA status
- `OTA_Health_Progress`: Current health check count
- `OTA_System_Healthy`: System health status
- `OTA_Last_Loop_Time`: Loop execution time

### **Serial Commands:**
- `otastatus`: Show current OTA validation status
- `otarollback`: Manually trigger rollback (with confirmation)
- `otavalidate`: Manually validate firmware (for testing)

## Safety Features

### **Multiple Fallback Mechanisms:**
1. **Health Check System**: Primary validation mechanism
2. **Watchdog Timer**: Hardware-level crash protection
3. **Loop Time Monitoring**: Endless loop detection
4. **System Health Checks**: WiFi and sensor monitoring
5. **Timeout Protection**: Maximum validation time limit

### **Rollback Triggers:**
- Insufficient health checks within timeout
- System becomes unhealthy
- Loop execution time exceeds limit
- Watchdog timeout
- Manual rollback request

## Benefits

### **Reliability:**
- Prevents bad firmware from being permanently installed
- Automatic recovery from firmware issues
- Multiple validation layers

### **Monitoring:**
- Real-time validation progress
- Comprehensive telemetry
- Easy debugging and status checking

### **Flexibility:**
- Configurable thresholds
- Manual override capabilities
- Testing and development support

## Usage Examples

### **Normal OTA Update:**
1. Firmware downloaded and installed
2. System reboots with new firmware
3. Health check system activates
4. 15 successful loops complete
5. Firmware marked as valid
6. Normal operation resumes

### **Failed OTA Update:**
1. Firmware downloaded and installed
2. System reboots with new firmware
3. Health check system activates
4. System becomes unhealthy or timeout occurs
5. Automatic rollback triggered
6. System reboots with previous firmware

### **Manual Commands:**
```bash
# Check OTA status
otastatus

# Manually validate firmware (for testing)
otavalidate

# Manually trigger rollback
otarollback
```

## Future Enhancements

### **NVS Persistence:**
- Save health check progress across reboots
- Persistent validation state
- Recovery from power failures during validation

### **Advanced Health Checks:**
- Memory usage monitoring
- Network connectivity tests
- Sensor calibration verification
- Performance benchmarking

### **Offline Mode Support:**
- Validation without WiFi connection
- Local health check storage
- Offline rollback capabilities

## Troubleshooting

### **Common Issues:**
1. **Validation Timeout**: Increase `MAX_VALIDATION_TIME` or reduce `REQUIRED_HEALTH_CHECKS`
2. **Loop Time Exceeded**: Check for blocking operations in main loop
3. **System Unhealthy**: Verify WiFi connection and sensor functionality
4. **Rollback Loops**: Check for persistent hardware issues

### **Debug Commands:**
- Use `otastatus` to monitor validation progress
- Check serial output for health check messages
- Monitor ThingsBoard telemetry for real-time status

## Conclusion

The OTA Health Check System provides enterprise-grade reliability for firmware updates, ensuring that only stable and healthy firmware versions are permanently installed. With automatic rollback capabilities and comprehensive monitoring, it significantly reduces the risk of firmware-related system failures.
