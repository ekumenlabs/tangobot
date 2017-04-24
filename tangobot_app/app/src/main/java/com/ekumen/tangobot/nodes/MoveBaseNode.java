/*
 * Copyright 2017 Ekumen, Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.ekumen.tangobot.nodes;

import org.apache.commons.logging.Log;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NativeNodeMain;
import org.ros.node.Node;

public class MoveBaseNode extends NativeNodeMain {
    public static final String NODE_NAME = "move_base";
    private Log mLog;

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
        mLog = connectedNode.getLog();
        super.onStart(connectedNode);
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        if (super.executeReturnCode != 0 && mLog != null) {
            mLog.error("Execute error code: " + Integer.toString(super.executeReturnCode), throwable);
        }
    }
}
