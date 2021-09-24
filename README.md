# RMAD-edge
RMAD-edge is the embedded firmware that runs on the wireless RMAD vibration and magnetic sensors.  These sensors are battery operated devises that during deep sleep mode listens to any activity on the vibration sensor. Once activity is detected, the device will transmitt data in real-time, using a SmartMeshIP wireless sensor network, to the RMAD-gateway. 

RMAD-edge is an EFM32 Giant Gecko application written in Simplicity Studio V3. RMAD-edge is coded for hardware revision 6, and is backwards compatible with hardware revisions 4 and 5. RMAD-edge software replaces all previous RMAD firmware versions (RMAD-railway and RMAD-geophones).
