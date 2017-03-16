package com.ekumen.tangobot.application;


import android.annotation.TargetApi;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.Bundle;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceActivity;
import android.preference.PreferenceFragment;
import android.preference.PreferenceManager;
import android.support.v7.widget.Toolbar;

/**
 * A {@link PreferenceActivity} that presents a set of application settings. On
 * handset devices, settings are presented as a single list. On tablets,
 * settings are split by category, with category headers shown to the left of
 * the list of settings.
 * <p>
 * See <a href="http://developer.android.com/design/patterns/settings.html">
 * Android Design: Settings</a> for design guidelines and the <a
 * href="http://developer.android.com/guide/topics/ui/settings.html">Settings
 * API Guide</a> for more information on developing a Settings UI.
 */
public abstract class MasterChooserSettingsActivity extends AppCompatPreferenceActivity implements
        SharedPreferences.OnSharedPreferenceChangeListener{

    protected SettingsPreferenceFragment mSettingsPreferenceFragment;
    protected SharedPreferences mSharedPref;

    /**
     * A preference value change listener that updates the preference's summary
     * to reflect its new value.
     */
    private static Preference.OnPreferenceChangeListener sBindPreferenceSummaryToValueListener = new Preference.OnPreferenceChangeListener() {
        @Override
        public boolean onPreferenceChange(Preference preference, Object value) {
            String stringValue = value.toString();

            if (preference instanceof ListPreference) {
                // For list preferences, look up the correct display value in
                // the preference's 'entries' list.
                ListPreference listPreference = (ListPreference) preference;
                int index = listPreference.findIndexOfValue(stringValue);

                // Set the summary to reflect the new value.
                preference.setSummary(
                        index >= 0
                                ? listPreference.getEntries()[index]
                                : null);

            } else {
                // For all other preferences, set the summary to the value's
                // simple string representation.
                preference.setSummary(stringValue);
            }
            return true;
        }
    };

    /**
     * Binds a preference's summary to its value. More specifically, when the
     * preference's value is changed, its summary (line of text below the
     * preference title) is updated to reflect the value. The summary is also
     * immediately updated upon calling this method. The exact display format is
     * dependent on the type of preference.
     *
     * @see #sBindPreferenceSummaryToValueListener
     */
    private static void bindPreferenceSummaryToValue(Preference preference) {
        // Set the listener to watch for value changes.
        preference.setOnPreferenceChangeListener(sBindPreferenceSummaryToValueListener);

        // Trigger the listener immediately with the preference's
        // current value.
        sBindPreferenceSummaryToValueListener.onPreferenceChange(preference,
                PreferenceManager
                        .getDefaultSharedPreferences(preference.getContext())
                        .getString(preference.getKey(), ""));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        mSharedPref = PreferenceManager.getDefaultSharedPreferences(getBaseContext());

        // This activity requires a method to force its start when "show settings" is off.
        boolean editSettings = getIntent().getBooleanExtra("edit_settings", false);
        boolean showSettings = mSharedPref.getBoolean(getString(R.string.pref_show_settings_on_startup_key), true);
        if (!showSettings && !editSettings) {
            onBackPressed();
        }

        setContentView(R.layout.settings_activity);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        mSharedPref.registerOnSharedPreferenceChangeListener(this);
        mSettingsPreferenceFragment = new SettingsPreferenceFragment();
        getFragmentManager().beginTransaction()
                .replace(R.id.fragment_container, mSettingsPreferenceFragment)
                .commit();
    }

    /**
     * This method stops fragment injection in malicious applications.
     * Make sure to deny any unknown fragments here.
     */
    protected boolean isValidFragment(String fragmentName) {
        return PreferenceFragment.class.getName().equals(fragmentName)
                || SettingsPreferenceFragment.class.getName().equals(fragmentName);
    }

    /**
     * Fragment showing settings preferences.
     */
    @TargetApi(Build.VERSION_CODES.KITKAT)
    public static class SettingsPreferenceFragment extends PreferenceFragment {
        @Override
        public void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            addPreferencesFromResource(R.xml.pref_settings);
            setHasOptionsMenu(true);

            // Bind the summaries of EditText/List/Dialog/Ringtone preferences
            // to their values. When their values change, their summaries are
            // updated to reflect the new value, per the Android Design
            // guidelines.
            bindPreferenceSummaryToValue(findPreference(getResources().getString(R.string.pref_master_uri_key)));
        }
    }

    @Override
    public void onBackPressed() {
        // Try to connect to Master by default.
        boolean editSettings = getIntent().getBooleanExtra("edit_settings", false);
        if (!editSettings) {
            Intent intent = new Intent();
            // This result and these extras are expected by the standard ROS Activity
            if (mSharedPref.getBoolean(getString(R.string.pref_master_is_local_key), false)) {
                intent.putExtra("ROS_MASTER_CREATE_NEW", true);
                intent.putExtra("ROS_MASTER_PRIVATE", false);
            } else {
                String uri = mSharedPref.getString(getResources().getString(R.string.pref_master_uri_key), "");
                intent.putExtra("ROS_MASTER_URI", uri);
            }

            setResult(RESULT_OK, intent);
        }
        super.onBackPressed();
    }
}
