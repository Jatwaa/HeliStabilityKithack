using UnityEngine;
using BepInEx;
using System.IO;

namespace KitHack.HeliTuningLab
{
    /// <summary>
    /// Live, per-frame helicopter state fed directly from RBInvestigation (or similar).
    /// This replaces the old RBHeliSnapshot + HeliSnapshotManager pattern.
    /// </summary>
    public class HeliRealtimeState
    {
        public Rigidbody rb;

        public float mass;
        public Vector3 inertiaTensor;
        public Quaternion inertiaTensorRotation;

        public Vector3 centerOfMassLocal;
        public Vector3 worldCenterOfMass;

        public Vector3 propWorldPos;
        public Vector3 thrustAxis;
        public float rotorRpm;
        public float pitchCollective;
        public float specificThrust;
        public float specificTorque;
        public float hoverThrust;

        public Vector3 angularVelocity;
        public Vector3 velocity;
        public float torque;

        public float collectiveLiftRatio;
        public float virtualTailArm;
        public float requiredAntiTorque;
        public Vector3 tailForceDirection;
    }

    [BepInPlugin("com.kithack.helituner", "Heli Tuning Lab", "4.0.0")]
    public class HeliTuningLabPlugin : BaseUnityPlugin
    {
        public enum FlightProfile
        {
            Aggressive, // A
            Balanced,   // B (default)
            Heavy       // C
        }

        private bool showWindow = true;
        private Rect windowRect = new Rect(50, 50, 560, 700);

        private bool autoMode = false;
        private bool pidEnabled = false;
        private bool forcesEnabled = false;

        private FlightProfile activeProfile = FlightProfile.Balanced;

        // Manual inputs
        private float rotorRpm = 4000f;
        private float collective = 0.3f;
        private float cyclicPitch = 0f;
        private float cyclicRoll = 0f;
        private float tailInput = 0f;

        // Auto / PID outputs
        private float autoCollective = 0f;
        private float autoCyclicPitch = 0f;
        private float autoCyclicRoll = 0f;
        private float autoTail = 0f;

        // PID controllers (Balanced realistic as baseline)
        private PID pidPitch = new PID(0.6f, 0.05f, 0.2f);
        private PID pidRoll = new PID(0.6f, 0.05f, 0.2f);
        private PID pidYaw = new PID(0.6f, 0.05f, 0.25f);
        private PID pidCollective = new PID(0.9f, 0.1f, 0.25f);

        // Auto-tuner & stability
        private float pitchErrAvg;
        private float rollErrAvg;
        private float yawErrAvg;
        private float stabilityScore;
        private string autoTunerState = "Idle";

        private float lastPitchErrorSign;
        private float lastRollErrorSign;
        private float lastYawErrorSign;
        private float timeSinceTune;

        // Last applied forces for monitor UI
        private Vector3 lastLiftForce;
        private Vector3 lastCyclicForce;
        private float lastTorqueComp;

        private string logFile;

        // Latest state instance used by this plugin (per-physics-tick)
        private HeliRealtimeState latestState;

        // Static direct-feed store, updated externally (e.g., from RBInvestigation)
        private static HeliRealtimeState externalState;
        private static bool hasExternalState;

        /// <summary>
        /// Called from your rotor/rigidbody analysis code once per physics frame.
        /// This is the "direct feed" entry point that replaces HeliSnapshotManager.
        /// </summary>
        public static void PushRealtimeState(HeliRealtimeState state)
        {
            externalState = state;
            hasExternalState = (state != null && state.rb != null);
        }

        private void Awake()
        {
            string folder = Path.Combine(Paths.GameRootPath, "BepInEx/HeliTunerLogs");
            Directory.CreateDirectory(folder);
            logFile = Path.Combine(folder, "HeliPID_FullPhysics_" + System.DateTime.Now.ToString("MMdd_HHmm") + ".log");
        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.F8))
                showWindow = !showWindow;
        }

        private void FixedUpdate()
        {
            // Build a live, per-physics-tick state from the latest direct feed + live RB data
            var s = BuildRealtimeState();

            latestState = s;

            if (s != null)
            {
                if (autoMode)
                {
                    if (pidEnabled)
                        ComputePIDControls(s);
                    else
                        ComputeSimpleAutoControls(s);

                    stabilityScore = ComputeStabilityScore(s);
                }
                else
                {
                    // Even in manual mode we can compute stability for display
                    stabilityScore = ComputeStabilityScore(s);
                }

                if (forcesEnabled)
                    ApplyPhysicsForces(s);
            }
        }

        private void Log(string msg)
        {
            try { File.AppendAllText(logFile, msg + "\n"); } catch { }
        }

        /// <summary>
        /// Combines externally-fed rotor data with live Rigidbody state.
        /// This is called every physics tick so cyclic and tail corrections react at full 50 Hz.
        /// </summary>
        private HeliRealtimeState BuildRealtimeState()
        {
            if (!hasExternalState || externalState == null || externalState.rb == null)
                return null;

            var src = externalState;
            var rb = src.rb;

            // Refresh RB-derived values every physics tick
            src.mass = rb.mass;
            src.inertiaTensor = rb.inertiaTensor;
            src.inertiaTensorRotation = rb.inertiaTensorRotation;
            src.centerOfMassLocal = rb.centerOfMass;
            src.worldCenterOfMass = rb.worldCenterOfMass;

            src.angularVelocity = rb.angularVelocity;
            src.velocity = rb.velocity;

            return src;
        }

        private void OnGUI()
        {
            if (showWindow)
                windowRect = GUI.Window(9283, windowRect, DrawWindow, "Heli Tuning Lab");
        }

        private void DrawWindow(int id)
        {
            GUILayout.BeginVertical("box");
            GUILayout.Label("Snapshot: " + (latestState != null ? "OK" : "None"));

            GUILayout.Space(5);

            // Profile button cycles A/B/C
            if (GUILayout.Button("Profile: " + activeProfile.ToString() + " (click to cycle)", GUILayout.Height(24)))
            {
                if (activeProfile == FlightProfile.Aggressive)
                    activeProfile = FlightProfile.Balanced;
                else if (activeProfile == FlightProfile.Balanced)
                    activeProfile = FlightProfile.Heavy;
                else
                    activeProfile = FlightProfile.Aggressive;
            }

            GUILayout.Space(5);

            Color prev = GUI.color;
            GUI.color = autoMode ? Color.green : Color.white;
            if (GUILayout.Button(autoMode ? "Auto Mode: ON" : "Auto Mode: OFF", GUILayout.Height(24)))
                autoMode = !autoMode;
            GUI.color = prev;

            GUI.color = pidEnabled ? Color.green : Color.white;
            if (GUILayout.Button(pidEnabled ? "PID Mode: ON (Auto-Tune)" : "PID Mode: OFF", GUILayout.Height(24)))
                pidEnabled = !pidEnabled;
            GUI.color = prev;

            GUILayout.Space(5);
            GUILayout.Label("TEST INPUTS");

            if (autoMode && latestState != null)
            {
                // Just display the last computed auto commands; they are updated in FixedUpdate.
                GUILayout.Label($"Collective (auto): {autoCollective:F3}");
                GUILayout.Label($"Pitch CMD: {autoCyclicPitch:F3}");
                GUILayout.Label($"Roll CMD: {autoCyclicRoll:F3}");
                GUILayout.Label($"Yaw CMD: {autoTail:F3}");
            }
            else
            {
                rotorRpm = Mathf.Round(GUILayout.HorizontalSlider(rotorRpm, 0, 8000));
                GUILayout.Label($"Rotor RPM: {rotorRpm}");

                collective = GUILayout.HorizontalSlider(collective, 0f, 1f);
                GUILayout.Label($"Collective: {collective:F3}");

                cyclicPitch = GUILayout.HorizontalSlider(cyclicPitch, -1f, 1f);
                GUILayout.Label($"Cyclic Pitch: {cyclicPitch:F3}");

                cyclicRoll = GUILayout.HorizontalSlider(cyclicRoll, -1f, 1f);
                GUILayout.Label($"Cyclic Roll: {cyclicRoll:F3}");

                tailInput = GUILayout.HorizontalSlider(tailInput, -1f, 1f);
                GUILayout.Label($"Tail Input: {tailInput:F3}");
            }

            GUILayout.Space(8);

            if (GUILayout.Button("Run Heli Test", GUILayout.Height(26)))
                Debug.Log("[HeliTuningLab] Run Heli Test clicked (placeholder).");

            prev = GUI.color;
            GUI.color = forcesEnabled ? Color.green : Color.white;
            if (GUILayout.Button(forcesEnabled ? "Applying Forces (ON)" : "Apply Physics Forces (OFF)", GUILayout.Height(26)))
                forcesEnabled = !forcesEnabled;
            GUI.color = prev;

            GUILayout.Space(8);

            DrawRealtimeMonitor();

            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        private void DrawRealtimeMonitor()
        {
            GUILayout.Label("=== REALTIME FORCE MONITOR ===");

            if (latestState == null)
            {
                GUILayout.Label("No state yet...");
                return;
            }

            GUILayout.Label($"Lift Force: {lastLiftForce.magnitude:F1} N ({lastLiftForce})");
            GUILayout.Label($"Cyclic Force: {lastCyclicForce.magnitude:F1} N ({lastCyclicForce})");
            GUILayout.Label($"Anti-Torque (proxy): {lastTorqueComp:F1}");
            GUILayout.Label($"Stability: {stabilityScore:F1} %");
            GUILayout.Label($"Auto-Tuner: {autoTunerState}");
        }

        // --- PID + Auto-tuner ---

        private void ComputePIDControls(HeliRealtimeState s)
        {
            float dt = Time.fixedDeltaTime > 0 ? Time.fixedDeltaTime : Time.deltaTime;
            Vector3 w = s.angularVelocity;

            float pitchError = -w.x;
            float rollError = -w.z;
            float yawError = -w.y;

            float profileGain = GetProfileGainMultiplier();

            autoCyclicPitch = Mathf.Clamp(pidPitch.Update(pitchError, dt) * profileGain, -1f, 1f);
            autoCyclicRoll = Mathf.Clamp(pidRoll.Update(rollError, dt) * profileGain, -1f, 1f);
            autoTail = Mathf.Clamp(pidYaw.Update(yawError, dt) * profileGain, -1f, 1f);

            float weight = s.mass * 9.81f;
            float collectiveError = weight - s.hoverThrust;
            autoCollective = Mathf.Clamp(pidCollective.Update(collectiveError, dt), 0f, 1f);

            AutoTuneGains(pitchError, rollError, yawError, dt, s);

            Log($"PID OUT profile:{activeProfile} pitch:{autoCyclicPitch:F3} roll:{autoCyclicRoll:F3} yaw:{autoTail:F3} coll:{autoCollective:F3} stab:{stabilityScore:F1}");
        }

        private void ComputeSimpleAutoControls(HeliRealtimeState s)
        {
            float weight = s.mass * 9.81f;
            autoCollective = Mathf.Clamp(weight / Mathf.Max(1f, s.hoverThrust), 0f, 1f);

            float profileGain = GetProfileGainMultiplier();

            autoCyclicPitch = Mathf.Clamp(-s.angularVelocity.x * 0.2f * profileGain, -1f, 1f);
            autoCyclicRoll = Mathf.Clamp(-s.angularVelocity.z * 0.2f * profileGain, -1f, 1f);
            autoTail = Mathf.Clamp(-(s.torque / 20000f) - s.angularVelocity.y * 0.1f * profileGain, -1f, 1f);

            stabilityScore = ComputeStabilityScore(s);
            autoTunerState = "Simple Auto";

            Log($"AUTO OUT profile:{activeProfile} pitch:{autoCyclicPitch:F3} roll:{autoCyclicRoll:F3} yaw:{autoTail:F3} coll:{autoCollective:F3} stab:{stabilityScore:F1}");
        }

        private float GetProfileGainMultiplier()
        {
            switch (activeProfile)
            {
                case FlightProfile.Aggressive:
                    return 1.6f; // sharper response
                case FlightProfile.Heavy:
                    return 0.65f; // smoother / heavier
                default:
                    return 1.0f; // Balanced baseline
            }
        }

        private void AutoTuneGains(float pitchError, float rollError, float yawError, float dt, HeliRealtimeState s)
        {
            timeSinceTune += dt;
            autoTunerState = "Auto-Tuning";

            float lerpFactor = Mathf.Clamp01(dt * 2f);
            pitchErrAvg = Mathf.Lerp(pitchErrAvg, Mathf.Abs(pitchError), lerpFactor);
            rollErrAvg = Mathf.Lerp(rollErrAvg, Mathf.Abs(rollError), lerpFactor);
            yawErrAvg = Mathf.Lerp(yawErrAvg, Mathf.Abs(yawError), lerpFactor);

            bool pitchFlip = Mathf.Sign(pitchError) != Mathf.Sign(lastPitchErrorSign) && Mathf.Abs(pitchError) > 0.02f;
            bool rollFlip = Mathf.Sign(rollError) != Mathf.Sign(lastRollErrorSign) && Mathf.Abs(rollError) > 0.02f;
            bool yawFlip = Mathf.Sign(yawError) != Mathf.Sign(lastYawErrorSign) && Mathf.Abs(yawError) > 0.02f;

            lastPitchErrorSign = Mathf.Sign(pitchError);
            lastRollErrorSign = Mathf.Sign(rollError);
            lastYawErrorSign = Mathf.Sign(yawError);

            const float hiErr = 0.25f;
            const float loErr = 0.03f;
            const float tuneRate = 0.35f;

            if (timeSinceTune > 0.25f)
            {
                TuneSingleAxis(pidPitch, pitchErrAvg, pitchFlip, hiErr, loErr, tuneRate, dt, "Pitch");
                TuneSingleAxis(pidRoll, rollErrAvg, rollFlip, hiErr, loErr, tuneRate, dt, "Roll");
                TuneSingleAxis(pidYaw, yawErrAvg, yawFlip, hiErr, loErr, tuneRate, dt, "Yaw");

                float weight = s.mass * 9.81f;
                float liftErrorNorm = Mathf.Abs(weight - s.hoverThrust) / Mathf.Max(1f, weight);
                if (liftErrorNorm > 0.15f)
                    pidCollective.Kp += tuneRate * dt;
                else if (liftErrorNorm < 0.02f)
                    pidCollective.Kp -= tuneRate * 0.5f * dt;

                pidCollective.ClampGains(0f, 5f);
                timeSinceTune = 0f;
            }
        }

        private void TuneSingleAxis(PID pid, float errAvg, bool flip, float hiErr, float loErr, float rate, float dt, string axisName)
        {
            if (errAvg > hiErr)
                pid.Kp += rate * dt;
            else if (errAvg < loErr)
                pid.Kp -= rate * 0.5f * dt;

            if (flip && errAvg > loErr)
            {
                pid.Kp *= 0.9f;
                pid.Kd += rate * 0.25f * dt;
            }

            pid.ClampGains(0f, 5f);
        }

        private float ComputeStabilityScore(HeliRealtimeState s)
        {
            float score = 100f;
            Vector3 w = s.angularVelocity;
            Vector3 v = s.velocity;

            score -= Mathf.Min(40f, w.magnitude * 40f);
            score -= Mathf.Min(30f, v.magnitude * 10f);

            float weight = s.mass * 9.81f;
            float liftErrorNorm = Mathf.Abs(weight - s.hoverThrust) / Mathf.Max(1f, weight);
            score -= Mathf.Min(30f, liftErrorNorm * 60f);

            return Mathf.Clamp(score, 0f, 100f);
        }

        // --- Force application using per-physics-frame realtime state ---

        private void ApplyPhysicsForces(HeliRealtimeState s)
        {
            if (s == null || s.rb == null)
                return;

            var rb = s.rb;

            float coll = autoMode ? autoCollective : collective;
            float pitchCmd = autoMode ? autoCyclicPitch : cyclicPitch;
            float rollCmd = autoMode ? autoCyclicRoll : cyclicRoll;
            float yawCmd = autoMode ? autoTail : tailInput;

            coll = Mathf.Clamp01(coll);

            float cycDemand = Mathf.Clamp01(Mathf.Abs(pitchCmd) + Mathf.Abs(rollCmd));
            float tailDemand = Mathf.Clamp01(Mathf.Abs(yawCmd));

            float baseLiftShare = 1f;
            float cycShare = cycDemand * 0.5f;
            float tailShare = tailDemand * 0.3f;

            float totalShare = baseLiftShare + cycShare + tailShare;
            float shareScale = 1f / Mathf.Max(totalShare, 1f);

            float Ttotal = s.hoverThrust * coll;
            float liftThrust = Ttotal * baseLiftShare * shareScale;
            float cycThrust = Ttotal * cycShare * shareScale;
            float yawThrust = Ttotal * tailShare * shareScale; // proxy-only for visualization/logging

            Vector3 liftDir = s.thrustAxis.sqrMagnitude > 0.001f
                ? s.thrustAxis.normalized
                : Vector3.up;

            Vector3 liftForce = liftDir * liftThrust;
            rb.AddForceAtPosition(liftForce, s.propWorldPos, ForceMode.Force);

            Vector3 right = Vector3.Cross(liftDir, Vector3.up).normalized;
            if (right.sqrMagnitude < 0.001f)
                right = Vector3.right;

            Vector3 forward = Vector3.Cross(right, liftDir).normalized;

            Vector2 cycVec = new Vector2(pitchCmd, rollCmd);
            if (cycVec.sqrMagnitude > 1e-4f)
                cycVec = cycVec.normalized;

            float pitchNorm = cycVec.x;
            float rollNorm = cycVec.y;

            Vector3 cyclicForce =
                forward * (pitchNorm * cycThrust) +
                right * (rollNorm * cycThrust);

            rb.AddForceAtPosition(cyclicForce, s.propWorldPos, ForceMode.Force);

            // --- Virtual tail rotor model ---
            float torqueComp = 0f;

            if (s.virtualTailArm > 0.01f &&
                Mathf.Abs(s.requiredAntiTorque) > 0.0001f &&
                s.tailForceDirection.sqrMagnitude > 0.001f)
            {
                // Force needed at the virtual tail arm to cancel yaw torque.
                float baseForce = s.requiredAntiTorque;
                float commandedForce = yawCmd * baseForce;

                Vector3 tailForce = s.tailForceDirection.normalized * commandedForce;

                // Imaginary tail behind CoM along -liftDir (simple tail boom approximation)
                Vector3 tailPos = s.worldCenterOfMass - liftDir * s.virtualTailArm;
                rb.AddForceAtPosition(tailForce, tailPos, ForceMode.Force);

                torqueComp = commandedForce; // log the commanded anti-torque "force"
            }
            else
            {
                // Fallback: older torque-based yaw compensation using rotor torque estimate.
                float rotorTorque = s.torque;
                torqueComp = yawCmd * rotorTorque;
                rb.AddTorque(liftDir * torqueComp, ForceMode.Force);
            }

            lastLiftForce = liftForce;
            lastCyclicForce = cyclicForce;
            lastTorqueComp = torqueComp;

            Log($"FORCES profile:{activeProfile} lift:{liftForce} cyc:{cyclicForce} torque:{torqueComp:F1} shares L:{liftThrust:F1} C:{cycThrust:F1} Y:{yawThrust:F1}");
        }
    }

    public class PID
    {
        public float Kp, Ki, Kd;
        private float integral;
        private float lastError;

        public PID(float kp, float ki, float kd)
        {
            Kp = kp; Ki = ki; Kd = kd;
        }

        public float Update(float error, float dt)
        {
            integral += error * dt;
            float derivative = dt > 0 ? (error - lastError) / dt : 0f;
            lastError = error;
            return Kp * error + Ki * integral + Kd * derivative;
        }

        public void ClampGains(float min, float max)
        {
            Kp = Mathf.Clamp(Kp, min, max);
            Ki = Mathf.Clamp(Ki, min * 0.1f, max * 0.5f);
            Kd = Mathf.Clamp(Kd, min, max);
        }
    }
}
