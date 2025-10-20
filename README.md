# BMEN6000E
Signal processing for medical devices

### Lab 1 - Temperature monitoring
The purpose of this experiment is to demonstrate an understanding of the **Wheatstone bridge circuit** by collecting real-time signal processing data using a _physiological (temperature) sensor_. By curve fitting the Steinhart-Hart equation, the measured resistance values are interpolated with known temperatures to calculate the calibration points, or Steinhart-Hart coefficients, in order to compute the temperature readings of the system. By evaluating all, 10-, and 100-time point intervals for the moving average at standard increments of temperature measured from the thermistor, the level of noise in the system can be observed to gauge the variability in hardware tolerances and data processing that ultimately influences the accuracy and precision of the results. 


### Lab 2 - Photoplethysmography (PPG)

The purpose of this experiment is to demonstrate an understanding of _photoplethysmography (PPG) sensors_ to measure **blood oxygenation, pulse rate (heart rate), and respiratory rate**. The red and infrared lights can be measured to estimate the hemoglobin oxygen saturation of arterial blood, represented as an AC signal that is superimposed on a DC signal. By taking the derivative and thresholding, SPO2 and heart rate can be calculated for event detection. By removing the DC component from the infrared signal and obtaining the inter-beat interval through the lag (shift), the autocorrelation can be calculated. Lastly, the Fourier transform can transform the infrared signal into a frequency domain to detect peaks, resulting in another method of computing heart rate. When comparing the library-computed heart rate, the rapid response, noise, and accuracy of the manually calculated heart rates can be evaluated. 

## Lab 3 - Glucometer