# PIC32MK
Several short (one page) test files in C for different peripheries and internal interfaces useful to develop any embedded application.

## 1. Few comments for all examples:
	a. The main idea is to share very simple, commented and complete examples.
	b. The PIC32MK is applying  an external HR clock 10 MHz.
	c. All tests are based on PIC32MK1024MCF064 but they could be easy adapted for different devices.
	d. Pay attention on the UART installation: pins initialization and  channel number (printf).
	e. The UART and its echo (included into all examples) is applying to communicate with test and verify if the PIC is working.
	f. Some test are in /*     */ (example UART).
	g. If you have any comments, improvements, please lets help us
	h. The ADC test does'nt apply the "REGbits" solution.
