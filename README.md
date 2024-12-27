# Measuring-Vitals-with-AVR-NTUA-BOARD

In the context of the Microcomputers Laboratory course at ECE NTUA, Ioannis Danias and I programmed the educational NTUA BOARD to simulate the measurement of a patient's temperature and blood pressure and to communicate with a web server for presenting the results.

The temperature is simulated using the DS18B20 sensor, and the blood pressure through the variation of potentiometer values. The results are displayed on an LCD screen on the board and on the web server.

Technologies we worked:

TWI (Two-Wire Interface)
PCA9555 (I2C I/O Expander)
LCD Display (using I2C)
USART (Universal Synchronous/Asynchronous Receiver/Transmitter)
One-Wire Protocol
Keypad scanning (matrix keypad)
GPIO (General Purpose Input/Output)

Special thanks to Kostas Frantzeskos and Kostas Psarras for their cooperation through the semester (we used a part of their code in terms of communicating with temp sensor)
