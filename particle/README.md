# PLANETARY STEWARDS GRANT - CONSERVIFY


---------- WIRING FOR PARTICLE ELECTRON ----------

GPS:						Utilizes UART on the TX and RX pins
								D6 used to power on/off GPS module (inverted logic)

ACCELEROMETER:	SPI over A2(SS), A3, A4, A5
								Configurable Interrupt over Electron's WKP Pin
								Reference LIS3DH.h for I2C setup info


ARDUCAM:				SCK => A3, MISO => A4, MOSI => A5, SS => A0
								SDA => D0, SCL => D1

MICROSD:				Primary SPI with DMA
								SCK(CLK) => A3, MISO(DO) => A4, MOSI(DI) => A5, SS(CS) => A1
