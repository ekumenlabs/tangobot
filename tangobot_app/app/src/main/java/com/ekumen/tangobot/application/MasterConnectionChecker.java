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

package com.ekumen.tangobot.application;

import android.os.AsyncTask;
import android.util.Log;

import org.ros.internal.node.client.MasterClient;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.namespace.GraphName;

import java.net.URI;
import java.net.URISyntaxException;

public class MasterConnectionChecker {

    public static final String TAG = "master_checker";
    protected UserHook userHook;
    protected String uri;

    private Object onSuccessPayload;

    /**
     * Lookup text for catching a ConnectionException when attempting to
     * connect to a master.
     */
    private static final String CONNECTION_EXCEPTION_TEXT = "ECONNREFUSED";

    /**
     * Lookup text for catching a UnknownHostException when attemping to
     * connect to a master.
     */
    private static final String UNKNOW_HOST_TEXT = "UnknownHost";

    public MasterConnectionChecker(String uri, UserHook hook) {
        this(uri, hook, null);
    }

    public MasterConnectionChecker(String uri, UserHook hook, Object onSuccessPayload) {
        userHook = hook;
        this.uri = uri;
        this.onSuccessPayload = onSuccessPayload;
    }

    public interface UserHook {
        void onSuccess(Object o);
        void onError(Throwable t);
    }

    public void runTest() {
        new AsyncConnection().executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
    }

    class AsyncConnection extends AsyncTask<Void, Void, Boolean> {
        // Make sure the URI can be parsed correctly and that the master is
        // reachable.
        @Override
        protected Boolean doInBackground(Void... params) {
            Throwable errorResult = null;
            try {
                Log.i(TAG, "Trying to reach master...");
                MasterClient masterClient = new MasterClient(new URI(uri));
                masterClient.getUri(GraphName.of("android/master_chooser_activity"));
                Log.i(TAG, "Connected!");
            } catch (URISyntaxException e) {
                Log.i(TAG, "Invalid URI.");
                errorResult = e;
            } catch (XmlRpcTimeoutException e) {
                Log.i(TAG, "Master unreachable!");
                errorResult = e;
            } catch (Exception e) {
                String exceptionMessage = e.getMessage();
                if(exceptionMessage.contains(CONNECTION_EXCEPTION_TEXT))
                    Log.i(TAG, "Unable to communicate with master!");
                else if(exceptionMessage.contains(UNKNOW_HOST_TEXT))
                    Log.i(TAG, "Unable to resolve URI hostname!");
                else
                    Log.i(TAG, "Communication error!");
                errorResult = e;
            }

            if (errorResult != null && userHook != null) {
                userHook.onError(errorResult);
            }
            // Return true if there was no error at all
            return errorResult == null;
        }

        @Override
        protected void onPostExecute(Boolean result) {
            if (result && userHook != null) {
                userHook.onSuccess(onSuccessPayload);
            }
        }
    }
}
