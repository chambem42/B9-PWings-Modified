using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace WingProcedural
{
    /// <summary>
    /// Reads the flap setting the rest of the vessel is already flying at, so variable-aspect
    /// wings follow the aircraft's real flaps when it has any. Under FAR that is the 0-3 vessel
    /// flap level; without FAR it is the stock control surfaces' Deploy toggle, which is on/off
    /// only and so reads as 0 or 3. Wings carry their own flap setting for craft that have no
    /// control surfaces to take it from - see WingProcedural's variable sweep region.
    /// </summary>
    public static class VariableSweep
    {
        private static Func<Vessel, int> farVesselFlapSetting;
        private static bool farChecked;

        /// <summary>
        /// Bind FerramAerospaceResearch.FARAPI.VesselFlapSetting(Vessel) once, from the assembly
        /// WingProcedural.CheckAssemblies has already matched. The plugin ships built without a FAR
        /// reference, hence reflection - but bound as a delegate, since this is read every frame
        /// and MethodInfo.Invoke would box the return and allocate an argument array each time.
        /// </summary>
        public static void InitFAR(Assembly far)
        {
            if (farChecked)
            {
                return;
            }

            farChecked = true;
            MethodInfo mi = far.GetType("FerramAerospaceResearch.FARAPI")
                              ?.GetMethod("VesselFlapSetting", BindingFlags.Public | BindingFlags.Static, null, new Type[] { typeof(Vessel) }, null);
            if (mi != null && mi.ReturnType == typeof(int))
            {
                farVesselFlapSetting = (Func<Vessel, int>)Delegate.CreateDelegate(typeof(Func<Vessel, int>), mi);
            }
            else
            {
                Debug.Log("B9 PWings - FARAPI.VesselFlapSetting not found, variable sweep falls back to the stock deploy toggle");
            }
        }

        // One entry per vessel, for the current frame only. Keyed by instance ID rather than the
        // Vessel itself, so a stale entry cannot pin a whole part tree alive across a scene load.
        private static readonly Dictionary<int, int> cachedLevels = new Dictionary<int, int>();
        private static int cachedFrame = -1;
        private static bool farThrowLogged;
        // FAR's last good answer per vessel (same keying), to stand in while it throws.
        private static readonly Dictionary<int, int> lastFarLevels = new Dictionary<int, int>();
        private static int farThrowsInARow;
        private const int FarThrowLimit = 300; // reads, about 5 s of a single craft

        /// <summary>
        /// Flap setting 0-3, or -1 when the vessel has no flap source of its own to read.
        /// Memoized per vessel per frame so a craft carrying dozens of wings only walks the part
        /// list once, however many swing-wing craft are loaded.
        /// </summary>
        public static int VesselFlapLevel(Vessel v)
        {
            if (v == null)
            {
                return -1;
            }

            if (cachedFrame != Time.frameCount)
            {
                cachedFrame = Time.frameCount;
                cachedLevels.Clear();
            }

            int id = v.GetInstanceID();
            if (!cachedLevels.TryGetValue(id, out int level))
            {
                level = Measure(v);
                cachedLevels[id] = level;
            }

            return level;
        }

        private static int Measure(Vessel v)
        {
            if (farVesselFlapSetting != null)
            {
                try
                {
                    // 0-3, or -1 when the craft has no flap-tagged surface at all
                    int level = farVesselFlapSetting(v);
                    level = level < 0 ? -1 : Math.Min(level, 3);
                    lastFarLevels[v.GetInstanceID()] = level;
                    farThrowsInARow = 0;
                    return level;
                }
                catch (Exception e)
                {
                    // Most likely a vessel caught mid-change (parts being destroyed or split off).
                    // Answer with what FAR said last, not the stock toggle: a different answer
                    // reads as the pilot moving the flaps, and would drive the wings and override
                    // the level set on them. Logged once, as it can repeat every frame. Only if
                    // FAR keeps throwing is it dropped for the stock toggle, for the session.
                    if (!farThrowLogged)
                    {
                        farThrowLogged = true;
                        Debug.Log("B9 PWings - FARAPI.VesselFlapSetting threw, holding the last flap setting it gave: " + e);
                    }

                    if (++farThrowsInARow < FarThrowLimit)
                    {
                        return lastFarLevels.TryGetValue(v.GetInstanceID(), out int last) ? last : -1;
                    }

                    Debug.Log("B9 PWings - FARAPI.VesselFlapSetting keeps throwing, falling back to the stock deploy toggle");
                    farVesselFlapSetting = null;
                }
            }

            bool any = false;
            for (int i = 0; i < v.parts.Count; ++i)
            {
                ModuleControlSurface cs = v.parts[i].Modules.GetModule<ModuleControlSurface>();
                if (cs == null)
                {
                    continue;
                }

                any = true;
                if (cs.deploy)
                {
                    return 3;
                }
            }

            return any ? 0 : -1;
        }
    }
}
