# Audio_Analyzer

<h3>An audio analyzer and controller for an Arduino-driven LED strip.</h3>

![image](https://github.com/StormyTalents/Audio_Analyzer/assets/98739389/bfcdb31c-3e54-425d-8a01-0f569bda3568)

<p>This application takes audio input from the microphone or speaker output, processes it, and then calculates certain audio metrics such as intensity gain, bass gain, average intensity, peak frequency, and estimated pitch. Coupled with the audio analyzer is an Arduino LED-controller, which I send serialized data about the changes in audio metrics in order to control a WS2812B LED-strip.</p>

<p>The audio analyzer is able to extract useful information from audio in real-time. When processing the data, it takes into account the non-linearities of the human ear and modifies the data accordingly, in order to extract the most noticeable audio features. Then, it determines what information is required by the LED-controller and uses bit-packing to serialize and send the data as quickly as possible to give the smallest possible response time. Currently, the estimated pitch is mapped to a point on the color spectrum and used to control the hue of the lights, and the bass gain is used to control the brightness of the lights.</p>
