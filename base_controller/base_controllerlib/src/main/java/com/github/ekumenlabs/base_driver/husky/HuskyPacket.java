package com.github.ekumenlabs.base_driver.husky;

import org.apache.commons.codec.binary.Hex;

/**
* @author jcerruti@creativa77.com (Julian Cerruti)
*/
public class HuskyPacket {
    public static final char TYPE_ENCODER_DATA = (char)0x8800;
    public static final char TYPE_ENCODER_DATA_RAW = (char)0x8801;

    byte soh;
    byte length;
    byte lengthComplement;
    byte version;
    int timestamp;
    byte flags;
    char messageType;
    byte stx;
    byte[] payload;
    char checksum;

    public byte getLength() {
        return length;
    }

    public int getTimestamp() {
        return timestamp;
    }

    public byte getFlags() {
        return flags;
    }

    public byte[] getPayload() {
        return payload;
    }

    public char getMessageType() {
        return messageType;
    }

    public byte getVersion() {
        return version;
    }

    @Override
    public String toString() {
        return "HuskyPacket{" +
            Integer.toHexString(messageType) + ":" +
            new String(Hex.encodeHex(payload)) +
                '}';
    }
}
