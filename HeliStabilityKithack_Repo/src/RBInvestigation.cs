using BepInEx;
using UnityEngine;
using System;
using System.IO;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using Vehicles.Logging;
using Vehicles.Analysis;
using GameEvents;
using UnityEngine.Events;
using Modules;
using Input = UnityEngine.Input;
using KitHack.HeliShared;

[BepInPlugin("com.kithack.rbinvestigation", "RB Investigation", "5.1.0")]
public class RBInvestigation : BaseUnityPlugin
{
    // Base folder for logs
    private const string DIR =
        @"C:\Program Files (x86)\Steam\steamapps\common\KitHack Model Club\BepInEx\RBInvestigation";

    private string logPath;

    private VehicleLogger activeLogger;
    private Vehicle activeVehicle;
    private VehicleAnalysis analysis;

    // UI
    private bool showUI = false;
    private Vector2 scrollPos;
    private string latestLogText = "Spawn a vehicle to begin logging...";
    private bool showVisualizer = true;

    // World markers
    private Vector3 worldCoM;
    private Vector3 worldCoT;
    private Vector3 worldCoL;
    private Vector3 worldCoP;
    private Vector3 worldCounterThrustPos;

    private bool hasCoM;
    private bool hasCoT;
    private bool hasCoL;
    private bool hasCoP;
    private bool hasCounterThrust;

    // Metrics
    private float deltaThrust = 0f;
    private float deltaLift = 0f;

    private float requiredGimbalAngle = 0f;
    private Vector3 gimbalPreviewDir = Vector3.zero;
    private bool hasGimbalPreview = false;

    // Counter thrust solver
    private Vector3 counterThrustDir = Vector3.zero;
    private float counterThrustMagnitude = 0f;
    private float requiredCounterThrustKN = 0f;
    private bool hasCounterThrustSolution = false;

    // Engine state tracking for event-style logs
    private readonly Dictionary<Engine, bool> lastRunningState = new Dictionary<Engine, bool>();
    private readonly Dictionary<Engine, bool> lastStallState = new Dictionary<Engine, bool>();

    // --------------------------------------------------------------------
    // Logging file helpers
    // --------------------------------------------------------------------
    private string GenerateTimestampedLogPath()
    {
        Directory.CreateDirectory(DIR);

        string timestamp = System.DateTime.Now.ToString("MM_dd_yyyy__HHmm");
        string path = Path.Combine(DIR, $"Snoopy_{timestamp}.log");

        int counter = 1;
        while (File.Exists(path))
        {
            path = Path.Combine(DIR, $"Snoopy_{timestamp}_{counter}.log");
            counter++;
        }

        return path;
    }

    // --------------------------------------------------------------------
    // Unity lifecycle
    // --------------------------------------------------------------------
    void Awake()
    {
        logPath = GenerateTimestampedLogPath();
        File.WriteAllText(logPath,
            $"=== RB Investigation v5.1 Log ({System.DateTime.Now}) ==={Environment.NewLine}");
        Logger.LogInfo($"RBInvestigation writing to: {logPath}");

        // Subscribe to vehicle spawn
        GameEvents.Vehicles.OnVehicleSpawned.AddListener(OnVehicleSpawned);
    }

    void OnEnable()
    {
        Camera.onPostRender += OnCameraPostRender;
    }

    void OnDisable()
    {
        Camera.onPostRender -= OnCameraPostRender;
    }

    void OnDestroy()
    {
        if (activeLogger != null)
            activeLogger.OnLogEntryAdded -= OnLogEntryAdded;

        Camera.onPostRender -= OnCameraPostRender;
        GameEvents.Vehicles.OnVehicleSpawned.RemoveListener(OnVehicleSpawned);
    }

    // --------------------------------------------------------------------
    // Helpers
    // --------------------------------------------------------------------

    /// <summary>
    /// Compute an approximate "ship throttle" as the average throttleTgt
    /// across all engines on the current vehicle. Returns -1 if no engines.
    /// </summary>
    private float GetVehicleThrottle()
    {
        if (activeVehicle == null || activeVehicle.parts == null)
            return -1f;

        float sum = 0f;
        int count = 0;

        foreach (var part in activeVehicle.parts)
        {
            if (part == null) continue;

            IEnumerable<Engine> enginesEnum;
            try
            {
                enginesEnum = part.GetModules<Engine>();
            }
            catch
            {
                continue;
            }

            if (enginesEnum == null) continue;

            foreach (var eng in enginesEnum)
            {
                if (eng == null) continue;
                try
                {
                    sum += eng.throttleTgt.Value;
                    count++;
                }
                catch { }
            }
        }

        if (count == 0)
            return -1f;

        return sum / count;
    }

    // --------------------------------------------------------------------
    // Vehicle hook
    // --------------------------------------------------------------------
    private void OnVehicleSpawned(Vehicle v)
    {
        activeVehicle = v;

        // Create analysis instance bound to this vehicle
        analysis = VehicleAnalysis.Create(v);
        analysis.CalculateStatics();

        // Hook VehicleLogger
        var logger = v.GetComponent<VehicleLogger>();
        if (logger == null)
        {
            latestLogText = "Vehicle spawned but NO VehicleLogger present!";
            File.AppendAllText(logPath, "VehicleLogger missing on spawned vehicle." + Environment.NewLine);
            return;
        }

        if (activeLogger != null)
            activeLogger.OnLogEntryAdded -= OnLogEntryAdded;

        activeLogger = logger;
        activeLogger.OnLogEntryAdded += OnLogEntryAdded;

        // Reset engine state tracking when a new vehicle spawns
        lastRunningState.Clear();
        lastStallState.Clear();

        // Log basic craft/vehicle metadata
        var metaSb = new StringBuilder();
        metaSb.AppendLine();
        metaSb.AppendLine($"=== New Vehicle Session: {v.name} at t={Time.time} ===");
        try
        {
            metaSb.AppendLine($"ScnCraftSpawnGuid: {v.ScnCraftSpawnGuid}");
        }
        catch { }

        // Per-part geometry snapshot (runtime, effectively equivalent to craft file)
        if (v.parts != null)
        {
            metaSb.AppendLine("[PART_GEOMETRY_SNAPSHOT]");
            foreach (var part in v.parts)
            {
                if (part == null) continue;
                try
                {
                    string pname = !string.IsNullOrEmpty(part.FullName) ? part.FullName : part.name;
                    Transform t = part.transform;
                    metaSb.AppendLine(
                        $"Part={pname} LocalPos={t.localPosition} LocalRot={t.localRotation.eulerAngles}");
                }
                catch { }
            }
        }

        File.AppendAllText(logPath, metaSb.ToString());
        latestLogText = $"Vehicle Spawned: {v.name}{Environment.NewLine}Logging Active.";
    }

    // --------------------------------------------------------------------
    // Vehicle logger hook
    // --------------------------------------------------------------------
    private void OnLogEntryAdded(VehicleLogEntry entry)
    {
        if (activeVehicle == null)
            return;

        var v = activeVehicle;
        float shipThrottle = GetVehicleThrottle();

        var sb = new StringBuilder();
        sb.AppendLine("=== RBInvestigation Frame Telemetry ===");
        sb.AppendLine($"Entry Type: {entry.GetType().Name}");
        sb.AppendLine($"Frame: {entry.frame}  UT: {entry.UT:F3}");
        sb.AppendLine($"Pos: {entry.aPos}");
        sb.AppendLine($"Vel: {entry.wVel}");
        sb.AppendLine($"Acc: {entry.wAcc}");
        sb.AppendLine($"G:   {entry.gLoad}");
        sb.AppendLine($"AngVel: {entry.angularVel}");
        sb.AppendLine($"AngAcc: {entry.angularAcc}");
        sb.AppendLine($"RawEntry: {entry.GetFullLogString()}");

        // Ship throttle
        if (shipThrottle >= 0f)
            sb.AppendLine($"\nShip Throttle (avg engine throttleTgt): {shipThrottle:F3}");
        else
            sb.AppendLine("\nShip Throttle: N/A (no engines)");

        // VehicleAnalysis snapshot (CoM / CoT / CoL / CoP and thrust)
        try
        {
            if (analysis != null)
            {
                sb.AppendLine("\n--- VEHICLE ANALYSIS ---");
                sb.AppendLine($"WorldCoM: {worldCoM}");
                sb.AppendLine($"WorldCoT: {worldCoT}");
                sb.AppendLine($"WorldCoL: {worldCoL}");
                sb.AppendLine($"WorldCoP: {worldCoP}");
                sb.AppendLine($"TotalThrust: {analysis.thrust.totalThrust:F2} N");
                sb.AppendLine($"ThrustAxis: {analysis.thrust.Waxis}");
            }
        }
        catch { }

        // Rigidbody snapshot (global vehicle RB)
        Rigidbody rb = null;
        try
        {
            rb = v.GetComponent<Rigidbody>();
            if (rb != null)
            {
                sb.AppendLine("\n--- RIGIDBODY ---");
                sb.AppendLine($"Mass: {rb.mass:F3}");
                sb.AppendLine($"CenterOfMass (local): {rb.centerOfMass}");
                sb.AppendLine($"WorldCenterOfMass: {rb.worldCenterOfMass}");
                sb.AppendLine($"Velocity: {rb.velocity}");
                sb.AppendLine($"AngularVelocity: {rb.angularVelocity}");
                sb.AppendLine($"InertiaTensor: {rb.inertiaTensor}");
                sb.AppendLine($"InertiaTensorRotation: {rb.inertiaTensorRotation.eulerAngles}");
            }
        }
        catch { }

        // Per-engine diagnostics
        sb.AppendLine("\n--- ENGINE DIAGNOSTICS ---");

        if (v.parts != null)
        {
            foreach (var part in v.parts)
            {
                if (part == null) continue;

                IEnumerable<Engine> enginesEnum;
                try
                {
                    enginesEnum = part.GetModules<Engine>();
                }
                catch
                {
                    continue;
                }

                var engines = enginesEnum?.ToList();
                if (engines == null || engines.Count == 0)
                    continue;

                foreach (var eng in engines)
                {
                    if (eng == null) continue;

                    string partName = "UnknownPart";
                    try
                    {
                        if (eng.Part != null && !string.IsNullOrEmpty(eng.Part.name))
                            partName = eng.Part.name;
                        else if (!string.IsNullOrEmpty(part.name))
                            partName = part.name;
                    }
                    catch { }

                    bool running = false;
                    bool stall = false;
                    float throttleTgt = 0f;
                    float limiter = 0f;
                    float fxPower = 0f;
                    double shaftPower = 0.0;

                    try { running = eng.Running; } catch { }
                    try { stall = eng.Stall; } catch { }
                    try { throttleTgt = eng.throttleTgt.Value; } catch { }
                    try { limiter = eng.throttleLimiter; } catch { }
                    try { fxPower = eng.fxPower.Value; } catch { }
                    try { shaftPower = eng.ShaftPower; } catch { }

                    sb.AppendLine($"\nEngine Part: {partName}");
                    sb.AppendLine($"Running:       {running}");
                    sb.AppendLine($"Stall:         {stall}");
                    sb.AppendLine($"ThrottleTgt:   {throttleTgt:F3}");
                    sb.AppendLine($"Limiter:       {limiter:F3}");
                    sb.AppendLine($"FxPower:       {fxPower:F3}");
                    sb.AppendLine($"ShaftPower:    {shaftPower:F2}");

                    // Engine power plugs
                    try
                    {
                        if (eng.powerPlug != null)
                        {
                            sb.AppendLine($"PowerPlug Amt: {eng.powerPlug.amount:F3}");
                            sb.AppendLine($"PowerPlug Flow:{eng.powerPlug.flow:F3}");
                        }
                    }
                    catch { }

                    try
                    {
                        if (eng.inputPlug != null)
                        {
                            sb.AppendLine($"InputPlug Amt: {eng.inputPlug.amount:F3}");
                            sb.AppendLine($"InputPlug Flow:{eng.inputPlug.flow:F3}");
                        }
                    }
                    catch { }

                    // Engine events (start/stop/stall transitions)
                    bool prevRunning;
                    if (lastRunningState.TryGetValue(eng, out prevRunning))
                    {
                        if (running && !prevRunning)
                            sb.AppendLine("[EngineState] STARTED");
                        else if (!running && prevRunning)
                            sb.AppendLine("[EngineState] STOPPED");
                    }
                    lastRunningState[eng] = running;

                    bool prevStall;
                    if (lastStallState.TryGetValue(eng, out prevStall))
                    {
                        if (stall && !prevStall)
                            sb.AppendLine("[EngineState] STALLED");
                        else if (!stall && prevStall)
                            sb.AppendLine("[EngineState] STALL CLEARED");
                    }
                    lastStallState[eng] = stall;
                }
            }
        }

        // Rotor / Propeller diagnostics (for collective/cyclic/tail modeling)
        sb.AppendLine("\n--- ROTOR / PROPELLER DIAGNOSTICS ---");

        // We'll also capture the first/primary rotor data for heli model estimates
        bool mainRotorFound = false;
        Vector3 mainRotorWorldPos = Vector3.zero;
        Vector3 mainRotorWorldAxis = Vector3.zero;
        float mainRotorRotSpeed = 0f;
        float mainRotorPitch = 0f;
        float mainRotorSpecThrust = 0f;
        float mainRotorSpecTorque = 0f;
        Vector3 mainRotorMomentArm = Vector3.zero;

        if (v.parts != null)
        {
            foreach (var part in v.parts)
            {
                if (part == null) continue;

                IEnumerable<Propeller> propsEnum;
                try
                {
                    propsEnum = part.GetModules<Propeller>();
                }
                catch
                {
                    continue;
                }

                var props = propsEnum?.ToList();
                if (props == null || props.Count == 0)
                    continue;

                string partName = "UnknownPart";
                try
                {
                    if (!string.IsNullOrEmpty(part.FullName))
                        partName = part.FullName;
                    else if (!string.IsNullOrEmpty(part.name))
                        partName = part.name;
                }
                catch { }

                foreach (var prop in props)
                {
                    if (prop == null) continue;

                    // Pick a transform representing the rotor hub
                    Transform hubTr = null;
                    try
                    {
                        if (prop.trfPropSpin != null && prop.trfPropSpin.transform != null)
                            hubTr = prop.trfPropSpin.transform;
                    }
                    catch { }

                    if (hubTr == null)
                    {
                        try
                        {
                            if (prop.trfPropFlip != null && prop.trfPropFlip.transform != null)
                                hubTr = prop.trfPropFlip.transform;
                        }
                        catch { }
                    }

                    if (hubTr == null)
                        hubTr = part.transform;

                    Vector3 worldPos = hubTr.position;
                    // By convention prop cfg uses rotationAxis = (0,0,1) in local space
                    Vector3 localAxis = Vector3.forward;
                    Vector3 worldAxis = hubTr.TransformDirection(localAxis);

                    float rotSpeedRaw = 0f;
                    float pitch = 0f;
                    bool useSim = false;
                    bool waterProp = false;
                    float specThrust = 0f;
                    float specTorque = 0f;
                    float propInertia = 0f;

                    try { rotSpeedRaw = prop.GetRotSpeed(); } catch { }
                    try { pitch = prop.pitch; } catch { }
                    try { useSim = prop.usePropSim; } catch { }
                    try { waterProp = prop.waterProp; } catch { }
                    try { specThrust = prop.specificThrust; } catch { }
                    try { specTorque = prop.specificTorque; } catch { }
                    try { propInertia = prop.propInertia; } catch { }

                    sb.AppendLine($"\nProp Part: {partName}");
                    sb.AppendLine($"WorldPos: {worldPos}");
                    sb.AppendLine($"WorldAxis: {worldAxis}");
                    sb.AppendLine($"RotSpeedRaw: {rotSpeedRaw:F3}");
                    sb.AppendLine($"Pitch: {pitch:F3}");
                    sb.AppendLine($"UsePropSim: {useSim}  WaterProp: {waterProp}");
                    sb.AppendLine($"SpecificThrust: {specThrust:F6}  SpecificTorque: {specTorque:F6}");
                    sb.AppendLine($"PropInertia: {propInertia:F3}");

                    if (hasCoM)
                    {
                        Vector3 r = worldPos - worldCoM;
                        sb.AppendLine($"MomentArm (r = propPos - CoM): {r}");

                        // Store one main rotor candidate for heli modeling.
                        if (!mainRotorFound)
                        {
                            mainRotorFound = true;
                            mainRotorWorldPos = worldPos;
                            mainRotorWorldAxis = worldAxis;
                            mainRotorRotSpeed = rotSpeedRaw;
                            mainRotorPitch = pitch;
                            mainRotorSpecThrust = specThrust;
                            mainRotorSpecTorque = specTorque;
                            mainRotorMomentArm = r;
                        }
                    }
                }
            }
        }

        // ----------------------------------------------------------------
        // HELICOPTER MODEL ESTIMATES (Collective / Cyclic / Tail + flip risk)
        // ----------------------------------------------------------------
        sb.AppendLine("\n--- HELI MODEL (ESTIMATED) ---");

        // Defaults if we cannot compute
        sb.AppendLine("Note: Estimates derived from prop.rotSpeed, pitch, specificThrust/ Torque and CoM/CoT. Units are relative, useful for curves and comparing frames.");

        if (mainRotorFound && rb != null && hasCoM)
        {
            // 1) Estimate thrust magnitude from rotor speed
            // Assume rotSpeedRaw is RPM; convert to rad/s
            float omega = mainRotorRotSpeed * Propeller.RPM_2_RDS;
            // Relative thrust/torque estimates (not absolute Newtons/Nm, but proportional)
            float thrustEst = mainRotorSpecThrust * omega * omega;
            float torqueEst = mainRotorSpecTorque * omega * omega;

            // Rotor axis points along rotation axis; assume thrust is opposite axis (pulling "up")
            Vector3 thrustDir = -mainRotorWorldAxis.normalized;
            Vector3 thrustVec = thrustDir * thrustEst;

            // 2) Collective: vertical component of thrust vs gravity
            Vector3 worldUp = Vector3.up; // approximate; for more accuracy we could use -Physics.gravity.normalized
            float collectiveComponent = Vector3.Dot(thrustVec, worldUp);
            float mass = rb.mass;
            float weightApprox = mass * 9.81f;
            float collectiveLiftRatio = weightApprox > 1e-3f ? (collectiveComponent / weightApprox) : 0f;

            // 3) Cyclic: torque from thrust about CoM (using rotor moment arm)
            Vector3 r = mainRotorMomentArm;
            Vector3 torqueVec = Vector3.Cross(r, thrustVec);

            // 4) Approx angular accelerations about the body axes.
            // Assume inertiaTensor.x ~ pitch, y ~ yaw, z ~ roll principal axes.
            Vector3 I = rb.inertiaTensor;
            float alphaPitch = Mathf.Abs(I.x) > 1e-4f ? torqueVec.x / I.x : 0f;
            float alphaYaw   = Mathf.Abs(I.y) > 1e-4f ? torqueVec.y / I.y : 0f;
            float alphaRoll  = Mathf.Abs(I.z) > 1e-4f ? torqueVec.z / I.z : 0f;

            // 5) Flip risk estimation:
            // Use a "danger" threshold of ~60 deg/s^2 (about 1.047 rad/s^2)
            float alphaThreshold = Mathf.Deg2Rad * 60f;

            float flipRiskPitch = alphaThreshold > 1e-4f ? Mathf.Abs(alphaPitch) / alphaThreshold : 0f;
            float flipRiskRoll  = alphaThreshold > 1e-4f ? Mathf.Abs(alphaRoll) / alphaThreshold : 0f;

            // Predict throttle where we would CROSS the threshold if thrust scales ~ linearly with throttle
            float throttle = Mathf.Max(0.0001f, shipThrottle);
            float predictedFlipThrottlePitch = -1f;
            float predictedFlipThrottleRoll  = -1f;

            if (Mathf.Abs(alphaPitch) > 1e-4f)
                predictedFlipThrottlePitch = throttle * (alphaThreshold / Mathf.Abs(alphaPitch));
            if (Mathf.Abs(alphaRoll) > 1e-4f)
                predictedFlipThrottleRoll = throttle * (alphaThreshold / Mathf.Abs(alphaRoll));

            // 6) Tail "curve": yaw authority from rotor torque.
            // torqueEst is a rotor reaction torque; plus we have torqueVec.y from off-center thrust.
            float totalYawTorqueEst = Mathf.Abs(torqueVec.y) + Mathf.Abs(torqueEst);

            float alphaYawFromTorque = Mathf.Abs(I.y) > 1e-4f ? totalYawTorqueEst / I.y : 0f;
            float tailRiskYaw = alphaThreshold > 1e-4f ? Mathf.Abs(alphaYawFromTorque) / alphaThreshold : 0f;

            // Log the derived "curves" for this frame
            sb.AppendLine($"\n[Collective]");
            sb.AppendLine($"RotorRPM: {mainRotorRotSpeed:F1}");
            sb.AppendLine($"Pitch (collective): {mainRotorPitch:F3}");
            sb.AppendLine($"ThrustEst (rel): {thrustEst:F3}");
            sb.AppendLine($"CollectiveLiftRatio â‰ˆ {collectiveLiftRatio:F3}  (1.0 â‰ˆ hover)");

            sb.AppendLine($"\n[Cyclic]");
            sb.AppendLine($"MomentArm r: {r}");
            sb.AppendLine($"TorqueVec (from thrust): {torqueVec}");
            sb.AppendLine($"alphaPitch (rad/s^2): {alphaPitch:F4}");
            sb.AppendLine($"alphaRoll  (rad/s^2): {alphaRoll:F4}");
            sb.AppendLine($"FlipRiskPitch (1.0 = ~60deg/s^2): {flipRiskPitch:F3}");
            sb.AppendLine($"FlipRiskRoll  (1.0 = ~60deg/s^2): {flipRiskRoll:F3}");

            if (predictedFlipThrottlePitch > 0f)
                sb.AppendLine($"PredictedFlipThrottle_Pitch â‰ˆ {predictedFlipThrottlePitch:F3}");
            else
                sb.AppendLine("PredictedFlipThrottle_Pitch: N/A");

            if (predictedFlipThrottleRoll > 0f)
                sb.AppendLine($"PredictedFlipThrottle_Roll  â‰ˆ {predictedFlipThrottleRoll:F3}");
            else
                sb.AppendLine("PredictedFlipThrottle_Roll: N/A");

            sb.AppendLine($"\n[Tail / Yaw]");
            sb.AppendLine($"RotorReactionTorqueEst (rel): {torqueEst:F6}");
            sb.AppendLine($"YawTorqueFromOffset (rel): {Mathf.Abs(torqueVec.y):F6}");
            sb.AppendLine($"alphaYaw (rad/s^2): {alphaYawFromTorque:F4}");
            sb.AppendLine($"TailRiskYaw (1.0 = ~60deg/s^2): {tailRiskYaw:F3}");
        }

        // Report an enriched heli snapshot for the 50 Hz Heli Tuning Lab, using the
        // same estimated values we just logged above.
        if (mainRotorFound && rb != null && hasCoM)
        {
            Vector3 worldUp = Vector3.up;
            float omega_model = mainRotorRotSpeed * Propeller.RPM_2_RDS;

            // Rebuild the relative thrust / torque estimates from the model
            float thrustEst_model = mainRotorSpecThrust * omega_model * omega_model;
            float torqueEst_model = mainRotorSpecTorque * omega_model * omega_model;

            Vector3 thrustDir_model = -mainRotorWorldAxis.normalized;
            Vector3 thrustVec_model = thrustDir_model * thrustEst_model;

            // Compute collective lift ratio again as thrust vs weight along up
            float mass_model = rb.mass;
            float weight_model = mass_model * 9.81f;
            float collectiveComponent_model = Vector3.Dot(thrustVec_model, worldUp);
            float collectiveLiftRatio_model =
                weight_model > 1e-3f ? (collectiveComponent_model / weight_model) : 0f;

            // Yaw torque estimate from reaction torque + offset torque
            float mainTorqueFromThrust = Mathf.Abs(torqueVec.y);
            float reactionTorqueFromSpec = Mathf.Abs(torqueEst_model);
            float totalYawTorqueEst = mainTorqueFromThrust + reactionTorqueFromSpec;

            // Virtual tail rotor model: simple lever arm behind CoM along -thrust axis.
            const float virtualTailArm = 5.0f; // meters; tweak as desired

            // Direction for the side force from the virtual tail rotor: perpendicular to
            // both world up and the thrust axis.
            Vector3 tailDir = Vector3.Cross(worldUp, thrustDir_model).normalized;
            if (tailDir.sqrMagnitude < 1e-4f)
                tailDir = Vector3.right;

            // F = Ï„ / L for a moment arm perpendicular to the torque axis.
            float requiredAntiTorque = virtualTailArm > 0.01f
                ? (totalYawTorqueEst / virtualTailArm)
                : 0f;

            var snap = new RBHeliSnapshot
            {
                rb = rb,

                mass = rb.mass,
                inertiaTensor = rb.inertiaTensor,
                inertiaTensorRotation = rb.inertiaTensorRotation,

                centerOfMassLocal = rb.centerOfMass,
                worldCenterOfMass = rb.worldCenterOfMass,

                propWorldPos = mainRotorWorldPos,
                thrustAxis   = thrustDir_model,

                rotorRpm        = mainRotorRotSpeed,
                pitchCollective = mainRotorPitch,

                specificThrust = mainRotorSpecThrust,
                specificTorque = mainRotorSpecTorque,

                hoverThrust = thrustEst_model,
                torque      = totalYawTorqueEst,

                angularVelocity = rb.angularVelocity,
                velocity        = rb.velocity,

                collectiveLiftRatio = collectiveLiftRatio_model,
                virtualTailArm      = virtualTailArm,
                requiredAntiTorque  = requiredAntiTorque,
                tailForceDirection  = tailDir
            };

            HeliSnapshotManager.ReportSnapshot(snap);
        }
        else
        {
            sb.AppendLine("No main rotor & CoM & rigidbody combo found; heli model not evaluated for this frame.");
        }

        File.AppendAllText(logPath, sb.ToString() + Environment.NewLine + Environment.NewLine);
        latestLogText = sb.ToString();
    }

    // --------------------------------------------------------------------
    // Update & analysis refresh
    // --------------------------------------------------------------------
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.F10))
            showUI = !showUI;

        if (Input.GetKeyDown(KeyCode.F9))
            showVisualizer = !showVisualizer;

        if (activeVehicle != null && analysis != null)
            RefreshAnalysis(activeVehicle);
    }

    private void RefreshAnalysis(Vehicle v)
    {
        if (analysis == null || analysis.vehicle != v)
        {
            analysis = VehicleAnalysis.Create(v);
            analysis.CalculateStatics();
        }

        analysis.dynamics.Calculate(v);
        analysis.thrust.Calculate(v);
        analysis.aerodynamics.Calculate(v);

        // World markers
        worldCoM = analysis.dynamics.WCoM;
        hasCoM = true;

        worldCoT = analysis.thrust.WCoT;
        hasCoT = (analysis.thrust.totalThrust > 0f);

        var stab = analysis.aerodynamics.StabilityTest;
        var aero = stab.neutralAoASimData;

        hasCoL = stab.HasData;
        hasCoP = stab.HasData;

        if (hasCoL) worldCoL = aero.wCoL;
        if (hasCoP) worldCoP = aero.wCoP;

        // Counter-thrust placement (mirror around CoM, XZ plane balance)
        hasCounterThrust = false;
        hasCounterThrustSolution = false;
        requiredCounterThrustKN = 0f;

        if (hasCoM && hasCoT)
        {
            worldCounterThrustPos = 2f * worldCoM - worldCoT;
            hasCounterThrust = true;

            Vector3 offset = worldCoM - worldCoT;
            Vector3 offsetXZ = new Vector3(offset.x, 0f, offset.z);

            if (offsetXZ.sqrMagnitude > 0.0001f)
            {
                counterThrustDir = offsetXZ.normalized;
                counterThrustMagnitude = offsetXZ.magnitude;
                hasCounterThrustSolution = true;

                float totalThrust = analysis.thrust.totalThrust;
                float dist = Mathf.Max(0.01f, Vector3.Distance(worldCoT, worldCounterThrustPos));

                requiredCounterThrustKN = (offsetXZ.magnitude / dist) * totalThrust;
            }
        }

        // Î” metrics
        deltaThrust = (hasCoM && hasCoT) ? Vector3.Distance(worldCoT, worldCoM) : 0f;
        deltaLift = (hasCoM && hasCoL) ? Vector3.Distance(worldCoL, worldCoM) : 0f;

        // Virtual gimbal preview
        hasGimbalPreview = false;

        if (hasCoM && hasCoT && analysis.thrust.totalThrust > 0.01f)
        {
            Vector3 axis = analysis.thrust.Waxis.normalized;
            Vector3 desired = (worldCoM - worldCoT).normalized;

            requiredGimbalAngle = Vector3.Angle(axis, desired);

            if (requiredGimbalAngle > 0.01f)
            {
                Vector3 rotAxis = Vector3.Cross(axis, desired);
                if (rotAxis.sqrMagnitude > 0.001f)
                {
                    Quaternion q = Quaternion.AngleAxis(requiredGimbalAngle, rotAxis.normalized);
                    gimbalPreviewDir = q * axis;
                    hasGimbalPreview = true;
                }
            }
        }
    }

    // --------------------------------------------------------------------
    // GL drawing
    // --------------------------------------------------------------------
    private void OnCameraPostRender(Camera cam)
    {
        if (!showVisualizer || activeVehicle == null)
            return;

        if (cam != Camera.main)
            return;

        GL.PushMatrix();
        GL.LoadProjectionMatrix(cam.projectionMatrix);
        GL.modelview = cam.worldToCameraMatrix;

        GL.Begin(GL.LINES);

        // CoM
        if (hasCoM)
        {
            GL.Color(Color.yellow);
            DrawCross(worldCoM, 0.4f);
        }

        // CoT
        if (hasCoT)
        {
            GL.Color(Color.red);
            DrawCross(worldCoT, 0.4f);
        }

        // CoT -> CoM correction vector
        if (hasCoM && hasCoT)
        {
            GL.Color(Color.cyan);
            GL.Vertex(worldCoT);
            GL.Vertex(worldCoM);
        }

        // Aero markers
        if (hasCoL)
        {
            GL.Color(Color.blue);
            DrawCross(worldCoL, 0.4f);
        }

        if (hasCoP)
        {
            GL.Color(Color.green);
            DrawCross(worldCoP, 0.4f);
        }

        // CoP -> CoL
        if (hasCoL && hasCoP)
        {
            GL.Color(Color.magenta);
            GL.Vertex(worldCoP);
            GL.Vertex(worldCoL);
        }

        // Counter thrust vector
        if (hasCounterThrust && hasCounterThrustSolution)
        {
            Vector3 basePos = worldCounterThrustPos;
            Vector3 dir = counterThrustDir.normalized;
            float scale = Mathf.Clamp(requiredCounterThrustKN / Mathf.Max(1f, analysis.thrust.totalThrust), 0.2f, 6f);

            Vector3 tip = basePos + dir * scale * 3f;

            GL.Color(new Color(0f, 1f, 1f));
            GL.Vertex(basePos);
            GL.Vertex(tip);

            Vector3 side = Vector3.Cross(dir, Vector3.up).normalized * 0.3f;
            GL.Vertex(tip);
            GL.Vertex(tip - dir * 0.6f + side);
            GL.Vertex(tip);
            GL.Vertex(tip - dir * 0.6f - side);

            DrawCross(basePos, 0.35f);
        }

        // Virtual gimbal vector
        if (hasGimbalPreview && hasCoT)
        {
            GL.Color(new Color(0.6f, 0f, 1f));
            GL.Vertex(worldCoT);
            GL.Vertex(worldCoT + gimbalPreviewDir * 3f);
        }

        GL.End();
        GL.PopMatrix();
    }

    private void DrawCross(Vector3 p, float r)
    {
        GL.Vertex(p + Vector3.right * r);
        GL.Vertex(p - Vector3.right * r);

        GL.Vertex(p + Vector3.up * r);
        GL.Vertex(p - Vector3.up * r);

        GL.Vertex(p + Vector3.forward * r);
        GL.Vertex(p - Vector3.forward * r);
    }

    // --------------------------------------------------------------------
    // GUI overlay
    // --------------------------------------------------------------------
    void OnGUI()
    {
        if (showVisualizer)
        {
            GUI.color = Color.white;

            float shipThrottle = GetVehicleThrottle();
            GUI.Label(new Rect(20, Screen.height - 190, 500, 25),
                $"Ship Throttle (avg): {(shipThrottle >= 0f ? shipThrottle.ToString("F3") : "N/A")}" );

            GUI.Label(new Rect(20, Screen.height - 165, 500, 25),
                $"Î” Thrust (CoTâ€“CoM): {deltaThrust:F3} m");

            GUI.Label(new Rect(20, Screen.height - 140, 500, 25),
                $"Î” Lift (CoLâ€“CoM): {deltaLift:F3} m");

            if (hasGimbalPreview)
            {
                GUI.Label(new Rect(20, Screen.height - 115, 500, 25),
                    $"Required Gimbal: {requiredGimbalAngle:F2}Â°");
            }

            if (hasCounterThrustSolution)
            {
                GUI.Label(new Rect(20, Screen.height - 90, 500, 25),
                    $"Counter Thrust Needed (XZ): {requiredCounterThrustKN:F2} kN");
            }
        }

        if (!showUI) return;

        GUI.backgroundColor = new Color(0, 0, 0, 0.65f);
        GUI.contentColor = Color.white;

        GUILayout.BeginArea(new Rect(20, 20, 600, 800), GUI.skin.window);
        GUILayout.Label("<b>RB Investigation Console</b>",
            new GUIStyle(GUI.skin.label) { richText = true, fontSize = 18 });

        scrollPos = GUILayout.BeginScrollView(scrollPos);

        GUILayout.TextArea(latestLogText,
            new GUIStyle(GUI.skin.textArea)
            {
                richText = true,
                fontSize = 14,
                stretchHeight = true
            });

        GUILayout.EndScrollView();

        GUILayout.Label("<i>F10 = console | F9 = visual overlay</i>",
            new GUIStyle(GUI.skin.label) { richText = true });

        GUILayout.EndArea();
    }
}
