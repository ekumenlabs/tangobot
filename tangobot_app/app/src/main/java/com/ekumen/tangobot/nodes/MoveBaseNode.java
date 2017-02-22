package com.ekumen.tangobot.nodes;


import org.apache.commons.logging.Log;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NativeNodeMain;
import org.ros.node.Node;

public class MoveBaseNode extends NativeNodeMain {
    public static final String NODE_NAME = "move_base";
    private Log log;

    public MoveBaseNode() {
        super("move_base_jni");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    protected native int execute(String rosMasterUri, String rosHostname, String nodeName, String[] remappingArguments);

    @Override
    protected native int shutdown();

    @Override
    public void onStart(ConnectedNode connectedNode) {
        log = connectedNode.getLog();
        super.onStart(connectedNode);
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        if (super.executeReturnCode != 0 && log != null) {
            log.error("Execute error code: " + Integer.toString(super.executeReturnCode));
        }
    }
}
