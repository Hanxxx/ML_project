/*-----------------------------------------------------------------------------
 * File: TermTest.java
 * Function: Simple console terminal program to demonstrate serial port
 *           interface for Java using the SerialPort class.
 *
 * Copyright (c) 1996,1997,1998 Solutions Consulting, All Rights Reserved.
 *---------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------
 * Solutions Consulting makes no representations or warranties about the
 * suitability of the software, either express or implied, including but
 * not limited to the implied warranties of merchantability, fitness for
 * a particular purpose, or non-infringement. Solutions Consulting shall
 * not be liable for any damages suffered by licensee as a result of
 * using, modifying or distributing this software or its derivatives.
 *---------------------------------------------------------------------------*/
import java.io.*;
import javax.comm.*;

public class TermTest {
	static String banner = "Copyright (c) 1996,1997,1998 Solutions Consulting, All Rights Reserved.";
	static String ver = "TermTest Ver 2.0 - Comm API";

	public static void main(String args[]) {
	SerialPort sp = null;
	String devName = "";

	if (args.length == 0){
		PrintUsage();
		System.exit(1);
	}
	else {
		devName = args[0];
	}
	
	CommPortIdentifier.addPortName(devName, CommPortIdentifier.PORT_SERIAL, null);
	try {
		CommPortIdentifier p = 
			CommPortIdentifier.getPortIdentifier(devName);
//System.out.println("openPort...");
		sp = (SerialPort)p.open("TermTest App", 1);
        sp.setSerialPortParams(9600, 
			javax.comm.SerialPort.DATABITS_8, 
			javax.comm.SerialPort.STOPBITS_1, 
			javax.comm.SerialPort.PARITY_NONE);
//System.out.println("sp.getName="+sp.getName());
//		sp.setInputBufferSize(1000);
//System.out.println("getInputBufferSize="+sp.getInputBufferSize());

		sp.setDTR(true);
		TermRcvTask rcv = new TermRcvTask(sp.getInputStream());
		TermSndTask snd = new TermSndTask(sp.getOutputStream());
		rcv.start();
		snd.start();
		System.out.println("\nUse ^C to exit...");
		do {
			try { Thread.sleep(10000); } catch (InterruptedException se) {}
			} while(true);
		}
	catch (Exception ioe) {
		System.out.println(ioe);
		System.exit(1);
		}
  	}

	static int PrintUsage() {
		System.out.println(ver);
		System.out.println(banner);
		System.out.println("\nThis is a very simple serial port terminal program written in Java.");
		System.out.println("It uses native methods to communicate with the serial port.");
		System.out.println("The parameters are 9600 bps, No Parity, 8 data bits, 1 stop bit.");
		System.out.println("\nEnter the serial device name as the first parameter.");
		System.out.println("To use COM2, enter this command: java TermTest COM2");
		System.out.println("\nNote: If you use this program with a modem you may");
		System.out.println("want to issue the ATE0 command to cancel modem echo");
		return 0;
	}
}

/*-----------------------------------------------------------------------------
 * This thread reads the Recieve buffer and displays the values on the
 * console as characters
 *---------------------------------------------------------------------------*/
class TermRcvTask extends Thread {

	InputStream spIn;

	TermRcvTask(InputStream in) throws IOException
		{
		spIn = in;
		}

	public void run() {
		int b;

		for (;;) {
			try {
				b = spIn.read();
				System.out.print((char)b);
			}
			catch (Exception e) {
				System.out.println("Error in TermRcvTask "+e);
		 	}
		}
	}
}

/*-----------------------------------------------------------------------------
 * This thread reads the keyboard for input and sends the keystrokes to
 * the serial port transmit buffer
 *---------------------------------------------------------------------------*/
class TermSndTask extends Thread {
	
	OutputStream spOut;

	TermSndTask(OutputStream out) throws IOException
		{
		spOut = out;
		}

	public void run() {
		int b;
		for (;;) {
			try {
				b = System.in.read(); // blocks if input not available
//				System.out.println("PutByte("+b+")");
				spOut.write((byte)b);
				}
			catch (Exception e) {
				System.out.println("Error in TermSndTask "+e);
				}
			}
		}
}