package com.ericwadkins.audioanalyzer;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.io.*;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Created by ericwadkins on 10/5/16.
 */
public class SerialComm {

    private final String portName;
    private final int baudRate;
    private final BlockingQueue<byte[]> outgoing = new LinkedBlockingQueue<>();

    private InputStream in = null;
    private OutputStream out = null;

    public SerialComm(String portName, int baudRate) {
        this.portName = portName;
        this.baudRate = baudRate;
    }

    /*public static void main(String[] args) {
        SerialComm comm = new SerialComm("/dev/cu.usbmodem1421", 9600);
        comm.start();
    }*/

    public void connect(String portName, int baudRate) throws Exception {
        System.out.println("Connecting to " + portName);
        CommPortIdentifier portIdentifier = null;
        try {
            portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
        } catch (gnu.io.NoSuchPortException e) {
            throw new IllegalArgumentException("No such port");
        }
        if (portIdentifier.isCurrentlyOwned()) {
            throw new IllegalStateException("Port is currently in use");
        }
        else {
            CommPort commPort = portIdentifier.open(this.getClass().getName(), 2000);

            if (commPort instanceof SerialPort) {
                SerialPort serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(baudRate,
                        SerialPort.DATABITS_8, SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);

                in = serialPort.getInputStream();
                out = serialPort.getOutputStream();

            }
            else {
                throw new IllegalArgumentException("Not a serial port");
            }
        }
        System.out.println("Connected with baud rate " + baudRate);
    }

    private InputStream getInputStream() {
        if (in != null) {
            return in;
        }
        throw new IllegalStateException("A connection has not been established yet.");
    }

    private OutputStream getOutputStream() {
        if (out != null) {
            return out;
        }
        throw new IllegalStateException("A connection has not been established yet.");
    }

    public void start() throws Exception {
        connect(portName, baudRate);

        final InputStream in = getInputStream();
        final OutputStream out = getOutputStream();

        Thread reader = new Thread(() -> {
            BufferedReader br = new BufferedReader(new InputStreamReader(in));
            String line;
            try {
                while ((line = br.readLine()) != null) {
                    try {
                        System.out.println(line);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            } catch (IOException e2) {
                e2.printStackTrace();
            }
        });

        Thread writer = new Thread(() -> {
            while (true) {
                try {
                    byte[] bytes = outgoing.take();
                    out.write(bytes);
                    out.flush();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        reader.start();
        writer.start();
    }

    public void send(byte[] bytes) {
        outgoing.add(bytes);
    }

}
