package com.ekumen.tangobot.application;

import android.content.SharedPreferences;
import android.support.design.widget.Snackbar;
import android.view.View;


public class SettingsActivity extends MasterChooserSettingsActivity {
    @Override
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
        if (key == getString(R.string.pref_master_is_local_key) ||
                key == getString(R.string.pref_master_uri_key)) {
            boolean previouslyStarted = mSharedPref.getBoolean(getString(R.string.pref_previously_started_key), false);
            if (previouslyStarted && mSettingsPreferenceFragment.getView() != null) {
                Snackbar snackbar = Snackbar.make(mSettingsPreferenceFragment.getView(), getString(R.string.snackbar_text_restart), Snackbar.LENGTH_INDEFINITE);
                View snackBarView = snackbar.getView();
                snackBarView.setBackgroundColor(getResources().getColor(android.R.color.holo_orange_dark));
                snackbar.show();
            }
        }
    }
}
