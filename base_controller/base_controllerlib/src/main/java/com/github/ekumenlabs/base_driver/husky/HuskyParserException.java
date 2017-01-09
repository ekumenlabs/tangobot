package com.github.ekumenlabs.base_driver.husky;

/**
* @author jcerruti@creativa77.com (Julian Cerruti)
*/
public class HuskyParserException extends java.lang.Exception {
    private final static String BASE_MESSAGE = "Exception parsing packet";
    public HuskyParserException(String reason) {
        super(BASE_MESSAGE + ": " + reason);
    }
}
