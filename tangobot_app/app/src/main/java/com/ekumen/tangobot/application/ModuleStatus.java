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

import android.app.Activity;
import android.graphics.drawable.Drawable;
import android.support.v4.graphics.drawable.DrawableCompat;
import android.widget.ImageView;

public class ModuleStatus {
    protected Activity activity;
    protected ImageView imageView;
    protected Status status;

    ModuleStatus(Activity activity, ImageView view) {
        this.activity = activity;
        this.imageView = view;
        this.status = Status.WAITING_FOR_START;
        switchStatusDisplay();
    }

    public void updateStatus(Status status) {
        if (this.status != status) {
            this.status = status;
            switchStatusDisplay();
        }
    }

    private void switchStatusDisplay() {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                int tintColor = 0;
                Drawable drawable = null;

                switch (status) {
                    case RUNNING:
                        drawable = activity.getResources().getDrawable(R.drawable.ic_check_circle_black_24dp);
                        tintColor = activity.getResources().getColor(android.R.color.holo_green_dark);
                        break;

                    case WAITING_FOR_START:
                        drawable = activity.getResources().getDrawable(R.drawable.ic_hourglass_empty_black_24dp);
                        tintColor = activity.getResources().getColor(android.R.color.holo_orange_dark);
                        break;

                    case ERROR:
                        drawable = activity.getResources().getDrawable(R.drawable.ic_cancel_black_24dp);
                        tintColor = activity.getResources().getColor(android.R.color.holo_red_dark);
                        break;

                    default:
                        break;
                }
                if (drawable != null) {
                    imageView.setImageDrawable(drawable);
                    drawable = DrawableCompat.wrap(imageView.getDrawable().mutate());
                    DrawableCompat.setTint(drawable, tintColor);
                }
            }
        });
    }

    enum Status {
        ERROR,
        WAITING_FOR_START,
        RUNNING
    }
}
