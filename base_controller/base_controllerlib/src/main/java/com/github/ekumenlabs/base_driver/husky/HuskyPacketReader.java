package com.github.ekumenlabs.base_driver.husky;

import org.apache.commons.codec.binary.Hex;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * @author jcerruti@creativa77.com (Julian Cerruti)
 */
public class HuskyPacketReader {
    public enum parserState {
        READY, PARTIAL
    };
    private parserState curState;
    private volatile byte[] stored;
    private int storedSize;

    private int storedIndex;
    private HuskyPacket parsedPacket;

    private static final Log log = LogFactory.getLog(HuskyPacketReader.class);

    public HuskyPacketReader() {
        curState = parserState.READY;

        stored = new byte[500]; // Should be big enough for the biggest packets
        storedSize = 0;
        storedIndex = 0;
    }

    /**
     * Parses incoming bytes into HuskyPacket objects
     * @return <code>null</code> if the packet is incomplete
     */
    public HuskyPacket parse(ByteBuffer buffer) throws com.github.ekumenlabs.base_driver.husky.HuskyParserException {
        // Samples:
        //  aa0df2000c030000000402550800de8f  (echo:out of range)
        //  aa0df20008040000000402550100d829
        //  aa0df200f7 | 030000000402550800ef4f (split message)
        //  aa0d | f200 d310 0000 0004 0255 0000 c42c

        int bufferSize = buffer.array().length;
        if(curState == parserState.READY) {
            // Initialize a new buffer copy
            buffer.get(stored, 0, bufferSize);
            storedSize = bufferSize;
            parsedPacket = new HuskyPacket();
        } else {
            // Add to the previous buffer and keep parsing
            buffer.get(stored, storedSize, bufferSize);
            storedSize += bufferSize;
        }

        ByteBuffer bufferParse = ByteBuffer.wrap(stored);
        bufferParse.order(ByteOrder.LITTLE_ENDIAN);
        curState = parserState.PARTIAL;
main:   while(storedIndex < storedSize) {
            switch(storedIndex) {
                case 0:     // SOH
                    log.debug("Parsing SOH");
                    parsedPacket.soh = bufferParse.get(storedIndex);
                    if(parsedPacket.soh != (byte)0xAA) {
                        // Reset state so that we can wait until the correct packet start.
                        storedSize = 0;
                        storedIndex = 0;
                        curState = parserState.READY;
                        throw new HuskyParserException("SOH byte doesn't match: " + parsedPacket.soh);
                    }
                    storedIndex += 1;
                    break;
                case 1:     // length
                    log.debug("Parsing length");
                    parsedPacket.length = bufferParse.get(storedIndex);
                    storedIndex += 1;
                    break;
                case 2:     // length complement
                    log.debug("Parsing length complement");
                    parsedPacket.lengthComplement = bufferParse.get(storedIndex);
                    // TODO: Verify complement against length
                    storedIndex += 1;
                    break;
                case 3:     // version
                    log.debug("Parsing version");
                    parsedPacket.version = bufferParse.get(storedIndex);
                    // TODO: Verify version
                    storedIndex += 1;
                    break;
                case 4:     // timestamp
                    log.debug("Parsing timestamp");
                    // Make sure we have enough data to parse this otherwise skip
                    if(storedSize - storedIndex < 4) {
                        break main;
                    }
                    parsedPacket.timestamp = bufferParse.getInt(storedIndex);
                    storedIndex += 4;
                    break;
                case 8:     // flags
                    log.debug("Parsing flags");
                    parsedPacket.flags = bufferParse.get(storedIndex);
                    storedIndex += 1;
                    break;
                case 9:     // message type
                    log.debug("Parsing message type");
                    parsedPacket.messageType = bufferParse.getChar(storedIndex);
                    if(storedSize - storedIndex < 2) {
                        break main;
                    }
                    storedIndex += 2;
                    break;
                case 11:     // stx
                    log.debug("Parsing STX");
                    parsedPacket.stx = bufferParse.get(storedIndex);
                    if(parsedPacket.stx != (byte)0x55) {
                        throw new HuskyParserException("STX byte doesn't match: " + parsedPacket.stx);
                    }
                    storedIndex += 1;
                    break;
                case 12:     // payload and CRC
                    log.debug("Parsing payload and CRC");
                    int payloadAndCrcLength = parsedPacket.length - 9;
                    if(storedSize - storedIndex < payloadAndCrcLength) {
                        break main;
                    }
                    parsedPacket.payload = new byte[payloadAndCrcLength - 2];
                    System.arraycopy(stored, 12, parsedPacket.payload, 0, parsedPacket.payload.length);
                    bufferParse.getChar(storedIndex + parsedPacket.payload.length);
                    // TODO: Verify checksum
                    storedIndex = 0;
                    curState = parserState.READY;
                    break main;
            }
        }

        if(curState == parserState.READY) {
            return parsedPacket;
        } else {
            return null;
        }
    }

    /**
     * Very simple command-line testing for parsing known packet values
     */
    public static void main(String args[]) throws java.lang.Exception {
        HuskyPacketReader reader = new HuskyPacketReader();

        // Try parsing entire packet in one shot (most common method)
        HuskyPacket packet = tryParse(reader, "aa0df2000c030000000402550800de8f");
        System.out.println("Packet = " + packet);

        // Try parsing packet in two passes
        System.out.println("Packet 1/2 = " + tryParse(reader, "aa0df200f7"));
        System.out.println("Packet 1/2 = " + tryParse(reader, "030000000402550800ef4f"));
    }
    private static HuskyPacket tryParse(HuskyPacketReader reader, String hexArrayString) throws java.lang.Exception {
        return reader.parse(ByteBuffer.wrap(Hex.decodeHex(hexArrayString.toCharArray())));
    }
}
