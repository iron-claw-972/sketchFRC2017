package org.usfirst.frc.team972.robot;
import com.fazecast.jSerialComm.*;

import java.io.InputStream;
import java.io.OutputStream;

/**
 * Created by Jody on 3/26/2017.
 */
public class TimeOfFlight {
    public SerialPort port;
    private InputStream is;
    private OutputStream os;
    String currentLine;

    public TimeOfFlight() {
        SerialPort ports[] = SerialPort.getCommPorts();
        for(int i=0; i<ports.length; i++){
        	System.out.println(ports[i].getDescriptivePortName());
            if(ports[i].getDescriptivePortName().toLowerCase().contains("due")) {
                port = ports[i];
                break;
            }
        }
        if(port == null) {
            System.out.println("Arduino Due can not be found.");
            return;
        } else {
            if(port.openPort()) {
                System.out.println("Arduino Due Port SUCCESS");
            } else {
                System.out.println("Arduino Due Port FAILED");
                return;
            }

            port.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 0, 0);
            port.setBaudRate(38400);

            is = port.getInputStream();
            os = port.getOutputStream();

        }

        StringBuilder sb = new StringBuilder();

        port.addDataListener(new SerialPortDataListener() {

            public int getListeningEvents() { return SerialPort.LISTENING_EVENT_DATA_AVAILABLE; }
            public void serialEvent(SerialPortEvent event)
            {
                if (event.getEventType() != SerialPort.LISTENING_EVENT_DATA_AVAILABLE)
                    return;
                byte[] newData = new byte[port.bytesAvailable()];
                int numRead = port.readBytes(newData, newData.length);
                for(byte b : newData) {
                    if((char)b == '\n') {
                        String s = sb.toString();
                        currentLine = s;
                        sb.setLength(0);
                    } else {
                        sb.append((char)b);
                    }
                }
            }
        });
    }

    public static boolean isNumeric(String str)
    {
        try
        {
            double d = Double.parseDouble(str);
        }
        catch(NumberFormatException nfe)
        {
            return false;
        }
        return true;
    }

    StringBuilder sb = new StringBuilder();
    byte[] bb = new byte[4];

    public double GetDataInMillimeters() {

        if((is != null) && (currentLine != null) && (isNumeric(currentLine))) {
            return Double.valueOf(currentLine);
        }

        return -1;
    }

}
