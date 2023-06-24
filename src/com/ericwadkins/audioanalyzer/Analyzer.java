package com.ericwadkins.audioanalyzer;

import javax.sound.sampled.*;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Created by ericwadkins on 10/3/16.
 */
public class Analyzer {

    // Display settings
    public static int width = 1024;
    public static int height = 375;
    public static final int COLOR_SPECTRUMS = 4;
    public static final int BASELINE = 50;

    public static final int DISPLAY_FPS = 10; // Lower if data is not showing properly

    // Audio settings
    public static final int SAMPLES_PER_SECOND = 8192; // Linearly shifts frequencies to the left (wider frequency range)
    public static final int UPDATES_PER_SECOND = 8; // Linearly decreases data points (-), decreases update time (+)

    // Analyzer settings
    public static final double MAX_VALUE = 250.0;
    public static final double BASS_UPPER_BOUND = 0.04 * (8192.0 / SAMPLES_PER_SECOND);

    public static final int STACK_SIZE = 10;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static final HashMap<Integer, Canvas> canvasMap = new HashMap<>();
    public static final HashMap<Integer, Frame> frameMap = new HashMap<>();
    public static int idCount = 0;
    public static int openCount = 0;

    public static long count = 0;

    public static void main(String args[]) {
        // Create format and get line info
        TargetDataLine line = null;
        AudioFormat format = new AudioFormat(SAMPLES_PER_SECOND, 16, 1, true, false);
        DataLine.Info info = new DataLine.Info(TargetDataLine.class, format);
        if (!AudioSystem.isLineSupported(info)) {
            throw new RuntimeException("Audio line not supported!");
        }

        // Obtain and open the line
        try {
            line = (TargetDataLine) AudioSystem.getLine(info);
            line.open(format);
        } catch (LineUnavailableException e) {
            e.printStackTrace();
        }

        SerialComm comm = new SerialComm("/dev/cu.usbmodem1421", 9600);
        try {
            comm.start();
        } catch (Exception e) {
            System.out.println("WARNING: Failed to create serial comm");
            comm = null;
        }

        System.out.println("\n***********************");
        System.out.println("Starting Audio Analyzer");
        System.out.println("***********************\n");
        start(line, format, comm);
    }

    public static void start(TargetDataLine line, AudioFormat format, SerialComm comm) {
        // Create display
        int display = createDisplay();

        // Create raw data array
        byte[] raw = new byte[(int) format.getSampleRate() * format.getFrameSize() / UPDATES_PER_SECOND];

        // Begin audio capture
        line.start();

        ArrayList<Frame> stack = new ArrayList<>(STACK_SIZE);
        int lastHue = -1;
        int lastSaturation = -1;
        int lastBrightness = -1;
        int sentCount = 0;
        while (true) {
            // Read data from the data line
            int n = line.read(raw, 0, raw.length);

            // Process the raw data
            Frame frame = process(raw, stack);

            // Analyze the processed data, modifies and returns frame
            frame = analyze(frame);

            FrameEvent event = new FrameEvent();

            int hue = (int) ((((double) frame.peakAverage * COLOR_SPECTRUMS / frame.processed.length) % 1) * 360);
            int saturation = 100; // full color
            int brightness = 50;
            if (frame.maxBassIntensityDifference > 0.05) {
                brightness = (int) (50 + frame.maxBassIntensityDifference * 50);
                System.out.println(frame.maxBassIntensityDifference);
                if (frame.maxBassIntensityDifference > 0.4) {
                    saturation = 0; // white
                }
            }

            if (hue != lastHue) {
                event.changeHue(hue);
                lastHue = hue;
            }
            if (saturation != lastSaturation) {
                event.changeSaturation(saturation);
                lastSaturation = saturation;
            }
            if (brightness != lastBrightness) {
                event.changeBrightness(brightness);
                lastBrightness = brightness;
            }

            // If something has changed
            if (event.getType() > 0) {
                sentCount++;
                byte[] bytes = event.getBytes();
                System.out.print("Packet " + sentCount + ": ");
                for (int i = bytes.length - 1; i >= 0; i--) {
                    byte b = bytes[i];
                    for (int j = 0; j < 8; j++) {
                        System.out.print((b & (1 << (7 - j))) != 0 ? 1 : 0);
                    }
                    System.out.print(" ");
                }
                System.out.println();
                /*System.out.print("Extracted ");
                int[] values = FrameEvent.extract(bytes);
                for (int v : values) {
                    System.out.print(v + " ");
                }
                System.out.println();
                System.out.println();*/

                // Send over serial comm
                if (comm != null) {
                    comm.send(bytes);
                }
            }


            // Add the frame to the stack for the next frame
            if (stack.size() == STACK_SIZE) {
                stack.remove(0);
            }
            stack.add(frame);

            // Update the data to be displayed
            updateDisplay(display, frame);
            count++;
            //System.out.println(count);
        }
    }

    public static Frame process(byte[] raw, ArrayList<Frame> stack) {
        Frame frame = new Frame();
        frame.stack = new ArrayList<>(stack);

        // Raw data
        frame.raw = raw;

        // Frequency data
        double[] frequencies = calculateFFT(raw);
        frame.frequencies = frequencies;

        // Processed data
        double[] processed = frequencies.clone();

        processed = applyFilter(processed, new double[]{1});
        processed = logScale(processed);
        processed = scaleRange(processed, 0, BASS_UPPER_BOUND, 0.5);
        for (int i = 0; i < stack.size(); i++) {
            processed = scaleSurrounding(processed, stack.get(i).peakAverage, 1 + Math.pow(0.5, stack.size() - i), 10);
        }
        // Normalize to range [0,1]
        processed = limit(processed, MAX_VALUE);
        processed = scaleRange(processed, 0, 1.0, 1.0 / MAX_VALUE);

        frame.processed = processed;

        return frame;
    }

    public static Frame analyze(Frame frame) {

        double[] processed = frame.processed;

        Frame last = null;
        if (frame.stack.size() > 0) {
            last = frame.stack.get(frame.stack.size() - 1);
        }

        double sum = 0.0;
        double bassSum = 0.0;
        double maxIntensity = 0.0;
        int maxFrequency = 0;
        double maxBassIntensity = 0.0;
        for (int i = 0; i < processed.length; i++) {
            sum += processed[i];
            if (processed[i] > maxIntensity) {
                maxIntensity = processed[i];
                maxFrequency = i;
            }
            if (i < processed.length * BASS_UPPER_BOUND) {
                bassSum += processed[i];
                if (processed[i] > maxBassIntensity) {
                    maxBassIntensity = processed[i];
                }
            }
        }

        double[] peakFiltered = peakFilter(processed, 2);
        ArrayList<Integer> peakFrequencies = new ArrayList<>();
        ArrayList<Double> peakIntensities = new ArrayList<>();
        for (int i = 0; i < peakFiltered.length; i++) {
            if (peakFiltered[i] > 0) {
                peakFrequencies.add(i);
                peakIntensities.add(peakFiltered[i]);
            }
        }
        for (int i = 1; i < peakIntensities.size(); i++) {
            int j = i;
            while (j > 0 && peakIntensities.get(j - 1) < peakIntensities.get(j)) {
                double temp = peakIntensities.get(j);
                peakIntensities.set(j, peakIntensities.get(j - 1));
                peakIntensities.set(j - 1, temp);
                int temp2 = peakFrequencies.get(j);
                peakFrequencies.set(j, peakFrequencies.get(j - 1));
                peakFrequencies.set(j - 1, temp2);
                j--;
            }
        }
        double maxPeakIntensity = 0.0;
        for (int i = 0; i < peakFrequencies.size(); i++) {
            if (peakIntensities.get(i) > maxPeakIntensity
                    && peakFrequencies.get(i) > (double) processed.length * BASS_UPPER_BOUND) {
                maxPeakIntensity = peakIntensities.get(i);
            }
        }
        ArrayList<Integer> peaksList = new ArrayList<>();
        for (int i = 0; i < peakFrequencies.size(); i++) {
            if (peakIntensities.get(i) > 0.7 * maxPeakIntensity
                    && peakFrequencies.get(i) > (double) processed.length * BASS_UPPER_BOUND) {
                peaksList.add(peakFrequencies.get(i));
            }
        }
        int[] peaks = new int[peaksList.size()];
        for (int i = 0; i < peaksList.size(); i++) {
            peaks[i] = peaksList.get(i);
        }
        frame.peaks = peaks;

        double peakTotal = 0.0;
        double peakWeightedTotal = 0.0;
        for (int p = 0; p < peaks.length; p++) {
            int i = peaks[p];
            double mult = (double) (peaks.length - p) / peaks.length;
            peakTotal += processed[i] * mult;
            peakWeightedTotal += processed[i] * mult * i;
        }
        if (peakTotal < 0.05) {
            peakWeightedTotal = 0;
        }
        frame.peakAverage = (int) (peakWeightedTotal / peakTotal);


        frame.averageIntensity = sum / processed.length;
        frame.averageBassIntensity = bassSum / (processed.length * BASS_UPPER_BOUND);
        frame.maxIntensity = maxIntensity;
        frame.maxFrequency = maxFrequency;
        frame.maxBassIntensity = maxBassIntensity;

        if (last != null) {
            frame.averageIntensityDifference = frame.averageIntensity - last.averageIntensity;
            frame.averageIntensityGain = (frame.averageIntensityDifference) / last.averageIntensity;

            frame.averageBassIntensityDifference = frame.averageBassIntensity - last.averageBassIntensity;
            frame.averageBassIntensityGain = (frame.averageBassIntensityDifference) / last.averageBassIntensity;

            frame.maxIntensityDifference = frame.maxIntensity - last.maxIntensity;
            frame.maxIntensityGain = (frame.maxIntensityDifference) / last.maxIntensity;

            frame.maxBassIntensityDifference = frame.maxBassIntensity - last.maxBassIntensity;
            frame.maxBassIntensityGain = (frame.maxBassIntensityDifference) / last.maxBassIntensity;
        }

        return frame;
    }

    public static double[] applyFilter(double[] data, double[] filter) {
        double mag = 0.0;
        for (int i = 0; i < filter.length; i++) {
            mag += Math.abs(filter[i]);
        }
        for (int i = 0; i < filter.length; i++) {
            filter[i] /= mag;
        }

        double[] processed = new double[data.length];
        for (int i = 0; i < data.length; i++) {
            double sum = 0.0;
            for (int j = -filter.length / 2; j <= (filter.length - 1) / 2; j++) {
                if (i + j >= 0 && i + j < data.length) {
                    sum += data[i + j] * filter[j + filter.length / 2];
                }
                else if (i + j < 0){
                    sum += data[0] * filter[j + filter.length / 2];
                }
                else {
                    sum += data[data.length - 1] * filter[j + filter.length / 2];
                }
            }
            processed[i] = Math.abs(sum);
        }
        return processed;
    }

    public static double[] logScale(double[] data) {
        double[] processed = new double[data.length];
        for (int i = 0; i < data.length; i++) {
            processed[i] = data[i] * Math.max(0, Math.log(i * 2));
        }
        return processed;
    }

    public static double[] scaleRange(double[] data, double low, double high, double mult) {
        double[] processed = new double[data.length];
        for (int i = 0; i < data.length; i++) {
            if (i >= (int) (low * data.length) && i < (int) (high * data.length)) {
                processed[i] = data[i] * mult;
            }
            else {
                processed[i] = data[i];
            }
        }
        return processed;
    }

    public static double[] scaleSurrounding(double[] data, int center, double mult, double slope) {
        double[] processed = new double[data.length];
        for (int i = 0; i < data.length; i++) {
            processed[i] = data[i] * Math.max(1, mult - Math.abs(center - i) / slope);
        }
        return processed;
    }

    public static double[] peakFilter(double[] data, int width) {
        double[] processed = new double[data.length];
        for (int i = 0; i < data.length; i++) {
            boolean peak = true;
            for (int j = -width; j <= width; j++) {
                if (i + j >= 0 && i + j < data.length) {
                    if (j < 0 && data[i + j] > data[i] || j > 0 && data[i + j] > data[i]) {
                        peak = false;
                        break;
                    }
                }
            }
            if (peak) {
                processed[i] = data[i];
            }
        }
        return processed;
    }

    public static double[] limit(double[] data, double limit) {
        double[] processed = new double[data.length];
        for (int i = 0; i < data.length; i++) {
            processed[i] = Math.min(limit, data[i]);
        }
        return processed;
    }

    public static double[] calculateFFT(byte[] signal) {
        final int mNumberOfFFTPoints = signal.length / 2;
        double mMaxFFTSample = 0.0;

        double temp;
        Complex[] y;
        Complex[] complexSignal = new Complex[mNumberOfFFTPoints];
        double[] absSignal = new double[mNumberOfFFTPoints/2];

        for(int i = 0; i < mNumberOfFFTPoints; i++){
            temp = (double)((signal[2*i] & 0xFF) | (signal[2*i+1] << 8)) / 32768.0F;
            complexSignal[i] = new Complex(temp,0.0);
        }

        y = FFT.fft(complexSignal);

        for(int i = 0; i < (mNumberOfFFTPoints/2); i++) {
            absSignal[i] = Math.sqrt(Math.pow(y[i].re(), 2) + Math.pow(y[i].im(), 2));
            if(absSignal[i] > mMaxFFTSample) {
                mMaxFFTSample = absSignal[i];
            }
        }

        return absSignal;

    }

    public static int createDisplay(String title) {
        idCount++;
        openCount++;
        final int id = idCount;
        JFrame mainFrame = new JFrame(title == null ? "Audio Analyzer " + id : title);
        mainFrame.setSize(width, height + 22);
        mainFrame.setLayout(new GridLayout(3, 1));
        JPanel controlPanel = new JPanel();
        controlPanel.setLayout(new FlowLayout());

        mainFrame.add(controlPanel);
        mainFrame.setVisible(true);
        Canvas canvas = new Canvas();
        canvas.setBackground(Color.BLACK);
        canvas.setSize(width, height);
        controlPanel.add(canvas);

        mainFrame.addComponentListener(new ComponentListener() {
            @Override
            public void componentResized(ComponentEvent e) {
                System.out.println(canvas.getWidth());
                System.out.println(canvas.getHeight());
                System.out.println(mainFrame.getWidth());
                System.out.println(mainFrame.getHeight());
                width = mainFrame.getWidth();
                height = mainFrame.getHeight();
                canvas.setSize(mainFrame.getWidth(), mainFrame.getHeight());
                canvas.repaint();
            }
            @Override
            public void componentMoved(ComponentEvent e) {
            }
            @Override
            public void componentShown(ComponentEvent e) {
            }
            @Override
            public void componentHidden(ComponentEvent e) {
            }
        });

        //mainFrame.pack();

        Graphics2D g = (Graphics2D) canvas.getGraphics();
        canvasMap.put(id, canvas);

        // Create display thread
        final Thread display = new Thread(new Runnable() {
            public void run() {
                boolean killed = false;
                while (!killed) {
                    if (frameMap.containsKey(id)) {
                        displayData(id, frameMap.get(id));
                    }
                    try {
                        Thread.sleep(1000 / DISPLAY_FPS);
                    } catch (InterruptedException e) {
                        killed = true;
                        openCount--;
                        if (openCount == 0) {
                            System.exit(0);
                        }
                    }
                }
            }
        });
        display.start();

        mainFrame.addWindowListener(new WindowAdapter() {
            public void windowClosing(WindowEvent windowEvent){
                display.interrupt(); // Use to kill thread
            }
        });
        KeyListener exit = new KeyListener() {
            @Override
            public void keyTyped(KeyEvent e) {
                if (e.getExtendedKeyCode() == 27) {
                    System.exit(0);
                }
            }
            @Override
            public void keyPressed(KeyEvent e) {
            }
            @Override
            public void keyReleased(KeyEvent e) {
            }
        };
        mainFrame.addKeyListener(exit);
        canvas.addKeyListener(exit);

        return id;
    }

    public static int createDisplay() {
        return createDisplay(null);
    }

    public static void displayData(int id, Frame frame) {
        Graphics2D g = (Graphics2D) canvasMap.get(id).getGraphics();
        g.setStroke(new BasicStroke(2));
        g.clearRect(0, 0, width, height);
        double xScale = (double) width / frame.processed.length;
        double yScale = height - BASELINE;
        int lastX = 0;
        int lastY = height - BASELINE;
        //String style = "rect";
        String style = "line";

        //g.setColor(Color.getHSBColor(0.0f, 1.0f, 0.2f));
        //g.fillRect(0, 0, (int) (data.length * xScale * BASS_UPPER_BOUND), height);

        if (frame.maxBassIntensityDifference > 0.05) {
            float brightness = (float) frame.maxBassIntensityDifference;
            g.setColor(Color.getHSBColor(0.0f, frame.maxBassIntensityDifference > 0.4 ? 1.0f : 0.0f, brightness));
            g.fillRect(0, 0, width, height);
        }
        g.setColor(Color.getHSBColor(0.0f, 0.0f, 1.0f));
        g.drawRect(0, (height - BASELINE), width, 1);

        g.setColor(Color.getHSBColor(0.0f, 0.0f, 0.5f));
        g.drawRect(0, (height - BASELINE) - (int) ((frame.maxIntensity + 0.01) * yScale),
                (int) (frame.processed.length * xScale), 3);

        g.setColor(Color.getHSBColor(0.0f, 1.0f, 0.5f));
        g.drawRect(0, (height - BASELINE) - (int) ((frame.maxBassIntensity + 0.01) * yScale),
                (int) (frame.processed.length * BASS_UPPER_BOUND * xScale), 3);

        g.setColor(Color.getHSBColor(0.0f, 0.0f, 0.5f));
        g.fillRect(0, (height - BASELINE) - (int) ((frame.averageIntensity + 0.01) * yScale),
                (int) (frame.processed.length * xScale), 3);

        g.setColor(Color.getHSBColor(0.0f, 1.0f, 0.5f));
        g.fillRect(0, (height - BASELINE) - (int) ((frame.averageBassIntensity + 0.01) * yScale),
                (int) (frame.processed.length * BASS_UPPER_BOUND * xScale), 3);

        for (int p = 0; p < frame.peaks.length; p++) {
            int i = frame.peaks[p];
            g.setColor(Color.getHSBColor(((float) i * COLOR_SPECTRUMS / frame.processed.length) % 1, 1.0f, 1.0f));
            g.fillRect((int) (i * xScale), (height - BASELINE) - (int) (frame.processed[i] * yScale),
                    (int) xScale, /*BASELINE +*/ (int) (frame.processed[i] * yScale));
        }

        g.setColor(Color.getHSBColor(((float) frame.peakAverage * COLOR_SPECTRUMS / frame.processed.length) % 1, 1.0f, 1.0f));
        g.drawRect((int) (frame.peakAverage * xScale - 1), 0, 3, height);
        //g.fillRect(0, 0, width, height);

        for (int i = 0; i < frame.processed.length; i++) {
            g.setColor(Color.getHSBColor(((float) i * COLOR_SPECTRUMS / frame.processed.length) % 1, 1.0f, 1.0f));
            int x = (int) (i * xScale);
            int y = (int) ((height - BASELINE) - Math.max(1, (int) (frame.processed[i] * yScale)));
            if (style == "line") {
                g.drawLine(lastX, lastY, x, y);
            }
            else if (style == "rect") {
                g.fillRect(x, y, (int) xScale, (height - BASELINE) - y);
            }
            lastX = x;
            lastY = y;
        }
    }

    public static void updateDisplay(int id, Frame frame) {
        frameMap.put(id, frame);
    }

    public static void printData(double[] data) {
        for (double d : data) {
            System.err.print(d + ", ");
        }
        System.err.println();
    }

    public static void printData(byte[] data) {
        for (byte d : data) {
            System.err.print(d + ", ");
        }
        System.err.println();
    }

    // Shell classes for data storage

    static class Frame {
        public byte[] raw;
        public double[] frequencies;
        public double[] processed;
        public double averageIntensity;
        public double averageBassIntensity;
        public double maxIntensity;
        public double maxFrequency;
        public double maxBassIntensity;
        public double averageIntensityDifference;
        public double averageBassIntensityDifference;
        public double maxIntensityDifference;
        public double maxBassIntensityDifference;
        public double averageIntensityGain;
        public double averageBassIntensityGain;
        public double maxIntensityGain;
        public double maxBassIntensityGain;
        public int[] peaks;
        public int peakAverage;
        public ArrayList<Frame> stack;
    }

    static class FrameEvent {
        private static final int HUE_CHANGE = 1;
        private static final int SATURATION_CHANGE = 2;
        private static final int BRIGHTNESS_CHANGE = 4;

        private static final int TYPE_LOAD_SIZE = 3;
        private static final int HUE_LOAD_SIZE = 9;
        private static final int SATURATION_LOAD_SIZE = 7;
        private static final int BRIGHTNESS_LOAD_SIZE = 7;

        private int type;
        private int hue;
        private int saturation;
        private int brightness;

        public void changeHue(int hue) {
            if (hue < 0 || hue > 359) {
                throw new IllegalArgumentException("Hue must be in the range 0-359");
            }
            this.hue = hue;
            type |= HUE_CHANGE;
        }

        public void changeSaturation(int saturation) {
            if (saturation < 0 || saturation > 100) {
                throw new IllegalArgumentException("Saturation must be in the range 0-100");
            }
            this.saturation = saturation;
            type |= SATURATION_CHANGE;
        }

        public void changeBrightness(int brightness) {
            if (brightness < 0 || brightness > 100) {
                throw new IllegalArgumentException("Brightness must be in the range 0-100");
            }
            this.brightness = brightness;
            type |= BRIGHTNESS_CHANGE;
        }

        public int getType() {
            return type;
        }

        public byte[] getBytes() {
            int bitCount = TYPE_LOAD_SIZE;
            boolean hueChange = (type & HUE_CHANGE) == HUE_CHANGE;
            boolean saturationChange = (type & SATURATION_CHANGE) == SATURATION_CHANGE;
            boolean brightnessChange = (type & BRIGHTNESS_CHANGE) == BRIGHTNESS_CHANGE;
            if (hueChange) {
                bitCount += HUE_LOAD_SIZE;
            }
            if (saturationChange) {
                bitCount += SATURATION_LOAD_SIZE;
            }
            if (brightnessChange) {
                bitCount += BRIGHTNESS_LOAD_SIZE;
            }
            int byteCount = bitCount / 8;
            if (bitCount % 8 != 0) {
                byteCount++;
            }
            //System.out.println("Event requires " + bitCount + " bits -> " + byteCount + " bytes");

            byte[] bytes = new byte[byteCount];

            int[] packed = new int[2];
            packed = pack(bytes, packed[0], packed[1], TYPE_LOAD_SIZE, type);
            if (hueChange) {
                packed = pack(bytes, packed[0], packed[1], HUE_LOAD_SIZE, hue);
            }
            if (saturationChange) {
                packed = pack(bytes, packed[0], packed[1], SATURATION_LOAD_SIZE, saturation);
            }
            if (brightnessChange) {
                packed = pack(bytes, packed[0], packed[1], BRIGHTNESS_LOAD_SIZE, brightness);
            }

            return bytes;
        }

        public static int[] extract(byte[] bytes) {
            int type = -1;
            int hue = -1;
            int saturation = -1;
            int brightness = -1;
            int[] unpacked = new int[3];
            unpacked = unpack(bytes, unpacked[1], unpacked[2], TYPE_LOAD_SIZE);
            type = unpacked[0];
            if ((type & HUE_CHANGE) == HUE_CHANGE) {
                unpacked = unpack(bytes, unpacked[1], unpacked[2], HUE_LOAD_SIZE);
                hue = unpacked[0];
            }
            if ((type & SATURATION_CHANGE) == SATURATION_CHANGE) {
                unpacked = unpack(bytes, unpacked[1], unpacked[2], SATURATION_LOAD_SIZE);
                saturation = unpacked[0];
            }
            if ((type & BRIGHTNESS_CHANGE) == BRIGHTNESS_CHANGE) {
                unpacked = unpack(bytes, unpacked[1], unpacked[2], BRIGHTNESS_LOAD_SIZE);
                brightness = unpacked[0];
            }

            return new int[] {type, hue, saturation, brightness};
        }

        private static int[] unpack(byte[] bytes, int byteIndex, int bitIndex, int loadSize) {
            int val = 0;
            for (int i = 0; i < loadSize; i++) {
                if (bitIndex > 7) {
                    bitIndex = 0;
                    byteIndex++;
                }
                val |= (((bytes[byteIndex] & (1 << bitIndex)) >> bitIndex) << i);
                bitIndex++;
            }
            return new int[] {val, byteIndex, bitIndex};
        }

        private static int[] pack(byte[] bytes, int byteIndex, int bitIndex, int loadSize, int val) {
            for (int i = 0; i < loadSize; i++) {
                if (bitIndex > 7) {
                    bitIndex = 0;
                    byteIndex++;
                }
                if ((val & (1 << i)) != 0) {
                    bytes[byteIndex] |= (1 << bitIndex);
                }
                bitIndex++;
            }
            return new int[] {byteIndex, bitIndex};
        }

        private static boolean isSet(byte b, int bit) {
            return (b & (1 << bit)) != 0;
        }

        private static byte set(byte b, int bit) {
            return (byte) (b | (1 << bit));
        }
    }

}
