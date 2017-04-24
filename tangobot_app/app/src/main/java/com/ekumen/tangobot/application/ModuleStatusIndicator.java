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
import android.content.res.TypedArray;
import android.graphics.drawable.Drawable;
import android.support.annotation.StyleableRes;
import android.support.v4.graphics.drawable.DrawableCompat;
import android.widget.ImageView;

/**
 * This class binds an {@link ImageView} to a {@link ModuleStatusIndicator}, allowing to change
 * it visual appearance according to its respective state (Running, Waiting, or Error).
 * It depends on indicators.xml, which specifies arrays with the image/ color to use in each case.
 */
public class ModuleStatusIndicator {
    protected Activity mActivity;
    protected ImageView mImageView;
    protected Status mStatus;

    private static final int RESOURCE_INDEX = 0;
    @StyleableRes private static final int COLOR_INDEX = 1;

    ModuleStatusIndicator(Activity activity, ImageView view) {
        mActivity = activity;
        mImageView = view;
        mStatus = Status.PAUSED;
        switchStatusDisplay();
    }

    public void updateStatus(Status status) {
        if (mStatus != status) {
            mStatus = status;
            switchStatusDisplay();
        }
    }

    /**
     * Look for the resource according to the case and apply it with its respective color.
     * Note: This function does not perform any checks as assumes the xml file is correct.
     * It should have one array for each state, and each array should have a drawable resource and a color resource (in that order).
     */
    private void switchStatusDisplay() {
        mActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                TypedArray img = null;

                switch (mStatus) {
                    case PAUSED:
                        img = mActivity.getResources().obtainTypedArray(R.array.status_paused);
                        break;

                    case OK:
                        img = mActivity.getResources().obtainTypedArray(R.array.status_running);
                        break;

                    case LOADING:
                        img = mActivity.getResources().obtainTypedArray(R.array.status_loading);
                        break;

                    case ERROR:
                        img = mActivity.getResources().obtainTypedArray(R.array.status_error);
                        break;
                }

                int color = img.getColor(COLOR_INDEX, mActivity.getResources().getColor(android.R.color.black));
                Drawable drawable = img.getDrawable(RESOURCE_INDEX);
                mImageView.setImageDrawable(drawable);
                drawable = DrawableCompat.wrap(mImageView.getDrawable().mutate());
                DrawableCompat.setTint(drawable, color);
                img.recycle();
            }
        });
    }

    enum Status {
        ERROR,
        PAUSED,
        LOADING,
        OK
    }
}
