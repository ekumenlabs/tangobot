/*
	Copyright (c) 2013 - York College of Pennsylvania, Patrick J. Martin
	The MIT License
	See license.txt for details.
*/

package com.github.ekumenlabs.base_driver.kobuki;
import org.apache.commons.codec.binary.Hex;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import java.nio.ByteBuffer;
import java.util.Arrays;

public class KobukiPacketReader {

    Log log = LogFactory.getLog(KobukiPacketReader.class);

    //kobuki sensors read from here to update. Accessible fields
	private byte[] sensorPacket = new byte[15];
	private byte[] inertialSensorPacket = new byte[4];
	private byte[] dockingIRPacket = new byte[3];
	private byte[] wheelCurrentPacket = new byte[4];
	private enum parserState {
		INIT, PARTIAL, COMPLETE;
	}
	//Fields for class
	private parserState curState;
	private volatile byte[] stored;
	private int bytesStillNeeded;
	private int totalSize;
	private int bytesStoredIndex;

	public KobukiPacketReader() {
		curState = parserState.INIT;
		stored = new byte[500]; // Should be big enough for the biggest packets
		bytesStillNeeded = 0;
		totalSize = 0;
		bytesStoredIndex = 0;
	}

	public final void newPacket(ByteBuffer buff) { // look for main headers in buffer
		if(curState == parserState.INIT || curState == parserState.COMPLETE) {
			for(int i=0; i < buff.array().length-2; i++){ // currently ignoring -86|85 split case... should be rare
				if(buff.array()[i] == -86 && buff.array()[i+1] == 85) { // Header 0 and Header 1 means packet
					curState = parserState.PARTIAL;
					if((buff.array().length-i) >= (buff.array()[i+2]+4)) { // Whole packet is in the buffer. +3 accounts for headers and size. Don't think this is right. Change(+4 for checksum)
						byte[] packet = new byte[byteToInt(buff.array()[i+2])+2]; // make byte array of proper size. +1 just in case!!!! Needs to be + 2 to account for checksum and length
						//If x + 2 is the length of the payload, the length of payload + length + check sum = length of payload + 2
						System.arraycopy(buff.array(), i + 2, packet, 0, byteToInt(buff.array()[i+2]) + 2); //copy in data needed
						if(verifyChecksum(byteToInt(buff.array()[i+2]) + 2, packet)) {
							System.arraycopy(packet, 1, packet, 0, packet.length - 1);
							goodPacket(byteToInt(buff.array()[i+2]) ,packet); // send goodPacket the array and length of array.  At this point, the packet contains ONLY the payload...
						} else {
							//packet not valid
						}
						curState = parserState.COMPLETE;
					} else {
						System.arraycopy(buff.array(), i+2, stored, 0, (buff.array().length-(i+2))); //Copy the partial packet into the stored array (only for partial packets)
						bytesStillNeeded = (buff.array()[i+2] + 2) - (buff.array().length-(i+2));
						totalSize = buff.array()[i+2] + 2;  //This is the payload length + 2.  This accounts for both the length of the payload byte as well as the checksum byte
						bytesStoredIndex = buff.array().length - (i+2); //Basically, in the case of a partial packet, this variable stores the length of the partial packet
					}
				}
			}
		} else {
			System.arraycopy(buff.array(), 0, stored, bytesStoredIndex, bytesStillNeeded);  //Copy in the "rest" of the packet
			if(verifyChecksum(totalSize, stored)) {
				System.arraycopy(stored, 1, stored, 0, stored.length - 1);  //Get the array without the checksum or length
				goodPacket(totalSize - 2, stored);
			} else {
				//packet not valid
			}
			curState = parserState.COMPLETE;
		}
	}

	private final boolean verifyChecksum(int length, byte[] packet) { //Method for verifying checksum

		byte checksum = 0;

		for(int i = 0; i < length - 1; i++) {  //length - 1 so that checksum is not included in total!

			checksum ^= packet[i];   //XOR's all values in packet EXCEPT: HEADERS AND CHECKSUM
		}

		if(packet[length - 1] == checksum) { //Ensures that calculated checksum is equal to the checksum found in the packet
			return true;
		} else {
			return false;
		}
	}

	private final void goodPacket(int length ,byte[] packet) { // Without first 2 headers, length, or checksum!
		//header
		//size
		//payload

		int curPlace = 0;
		byte[] toSend = new byte[100];
		System.arraycopy(packet, 0, toSend, 0, (packet[1]+2));
		curPlace = packet[1] + 2;
		sortParts(toSend);

		for(int i = 0; i < 6; i++) { // seven total packets six left
			 Arrays.fill(toSend, (byte)0);
			 //Changed from curPlace + 1 to curPlace. Changed last field from curPlace + 2 to packet[curPlace + 1]
			 System.arraycopy(packet, curPlace, toSend, 0, packet[curPlace+1] + 2); //Curplace is current index that needs to be accessed. packet[curPlace + 1] + 2 accounts for the size of the payload AND the index + payload size bytes
			 curPlace = curPlace + packet[curPlace+1] + 2; //Essentially, the index of the element we just accessed. + the length of the payload + 2 for the index and payload size
			 sortParts(toSend);  //Sorts packet array into individual packets (see method)
		}
	}

	private final void sortParts(byte[] packet){
		switch(packet[0]){ // Sort by packet[0] (The index of the packet. See kobuki driver page for more info).
			case 1:
				sensorPacket(packet); // Sorts sensor packet
				break;
			case 3:
				dockingIRPacket(packet); //sorts dockingIR packet
				break;
			case 4:
				inertialSensorPacket(packet); //Sorts inertial sensor packet
				break;
			case 6:
				wheelCurrentPacket(packet); //Sorts wheel current packet
				break;
			default:
				break;
		}
	}

	private final int byteToInt(byte b) {
		return b & 0xFF;
	}

	public final void sensorPacket(byte[] packet) { // Store sensor values from packet
		System.arraycopy(packet, 2, sensorPacket, 0, 15); //Copy packet from incoming byte array
	}

	private final void inertialSensorPacket(byte[] packet) {
		System.arraycopy(packet, 2, inertialSensorPacket, 0, 4); //Copy packet from incoming byte array
	}

	private final void dockingIRPacket(byte[] packet) {  //Sorts the IR docking packet
		System.arraycopy(packet, 2, dockingIRPacket, 0, 3); //Copy packet from incoming byte array
	}

	private final void wheelCurrentPacket(byte[] packet) {  //Sorts wheel current data packet
		System.arraycopy(packet, 2, wheelCurrentPacket, 0, 2); //Copy packet from incoming byte array
	}

	/**
	 * @return the sensorPacket
	 */
	public final byte[] getSensorPacket() {
		return sensorPacket;
	}

	/**
	 * @return the inertialSensorPacket
	 */
	public final byte[] getInertialSensorPacket() {
		return inertialSensorPacket;
	}

	/**
	 * @return the dockingIRPacket
	 */
	public final byte[] getDockingIRPacket() {
		return dockingIRPacket;
	}

	/**
	 * @return the wheelCurrentPacket
	 */
	public final byte[] getWheelCurrentPacket() {
		return wheelCurrentPacket;
	}
}
