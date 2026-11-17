using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace WingProcedural
{
    /// <summary>
    /// Moves the editor's center of mass and center of lift with the sweep preview. The preview
    /// turns only the models, not the part transforms (see SweepPreviewAllowed), and both markers
    /// work from the part transforms: the CoM from Part.CoMOffset, the stock CoL from
    /// Part.CoLOffset, and FAR's CoL from its wing model's centroid. So while the preview is off
    /// neutral, the wing and every part it carries get those points turned about the pivot with
    /// the models, and they go back exactly as they were once it returns to neutral.
    ///
    /// None of these values are saved with the craft. Anything else that rewrites one while the
    /// preview is up (B9 recomputing a carried control surface, a part switcher) wins: its value
    /// becomes the new neutral, and restoring leaves it alone.
    ///
    /// Only points move. Lift direction and size stay as they are, so a fold preview does not tilt
    /// the wing's lift; under FAR a swept wing also gets its mid-chord sweep turned with it, which
    /// is what it flies with. Its span and area stay put, since FAR derives wing mass from area.
    ///
    /// A movable wing carried by this one poses its own part and everything under it, about its
    /// own pivot, and is left out here - two previews writing the same offsets would each take
    /// the other's value as a new neutral and turn it again, every frame. So what such a wing
    /// carries does not follow this wing's preview.
    /// </summary>
    internal sealed class SweepPreviewBalance
    {
        private sealed class Entry
        {
            public Part part;
            public WingProcedural wing; // null when the part is not a procedural wing
            public Vector3 com, comSet;
            public Vector3 col, colSet;
            public PartModule far; // FARWingAerodynamicModel (or a subclass), or null
            public Vector3 farRoot, farRootSet;
            public double farSweep, farSweepSet;
            public bool farPending; // posed, but FAR's wing interactions not rebuilt yet
        }

        private readonly List<Entry> entries = new List<Entry>();
        private readonly List<WingProcedural> leftOut = new List<WingProcedural>();
        private float applied;
        private Quaternion appliedRotation = Quaternion.identity;
        private bool appliedPlanform;

        /// <summary>
        /// Pose the wing's balance points for a preview at deg (0 restores them). Returns true when
        /// a FAR wing model changed, which FAR only notices on a ship-modified event - and which
        /// wants FinishFar first.
        /// </summary>
        public bool Apply(Part wing, float deg, Quaternion rotation, bool sweepsPlanform)
        {
            if (deg == 0f)
            {
                return Restore();
            }

            bool rewritten = false;
            if (entries.Count == 0 || !StillValid())
            {
                // First pose, or a part went away or became (or stopped being) movable under the
                // preview: start over from what is there now.
                Restore();
                Capture(wing);
                rewritten = true;
            }
            else
            {
                for (int i = 0; i < entries.Count; ++i)
                {
                    rewritten |= TakeExternalChanges(entries[i]);
                }
            }

            // Switching between sweep and fold turns about another axis at the same angle.
            if (deg == applied && rotation == appliedRotation && sweepsPlanform == appliedPlanform && !rewritten)
            {
                return false;
            }

            applied = deg;
            appliedRotation = rotation;
            appliedPlanform = sweepsPlanform;
            Transform pivot = wing.transform;
            bool farChanged = false;
            for (int i = 0; i < entries.Count; ++i)
            {
                Entry e = entries[i];
                Transform t = e.part.transform;
                if (e.part == wing)
                {
                    e.comSet = rotation * e.com;
                    e.colSet = rotation * e.col;
                }
                else
                {
                    Vector3 com = Turn(pivot, rotation, t.position + t.rotation * e.com);
                    e.comSet = Quaternion.Inverse(t.rotation) * (com - t.position);
                    e.colSet = t.InverseTransformPoint(Turn(pivot, rotation, t.TransformPoint(e.col)));
                }

                e.part.CoMOffset = e.comSet;
                e.part.CoLOffset = e.colSet;

                if (e.far != null)
                {
                    PoseFar(e, pivot, rotation, e.part == wing, e.part == wing && sweepsPlanform ? deg : 0f);
                    farChanged = true;
                }
            }

            return farChanged;
        }

        /// <summary>
        /// Rebuild FAR's wing interactions for the posed wings, which hold their own copy of the
        /// root offset. Too slow to do on every step of a slider drag, so it waits until the slider
        /// stops, just ahead of the ship-modified event FAR recomputes its CoL on.
        /// </summary>
        public void FinishFar()
        {
            for (int i = 0; i < entries.Count; ++i)
            {
                Entry e = entries[i];
                if (e.farPending && e.part != null)
                {
                    farStart.Invoke(e.far, null);
                }

                e.farPending = false;
            }
        }

        /// <summary>Put everything back at neutral. Returns true when a FAR wing model changed.</summary>
        public bool Restore()
        {
            bool farChanged = false;
            for (int i = 0; i < entries.Count; ++i)
            {
                Entry e = entries[i];
                if (e.part == null)
                {
                    continue;
                }

                if (e.part.CoMOffset == e.comSet)
                {
                    e.part.CoMOffset = e.com;
                }

                if (e.part.CoLOffset == e.colSet)
                {
                    e.part.CoLOffset = e.col;
                }

                if (e.far != null)
                {
                    bool changed = e.farPending;
                    if (FarRoot(e.far) == e.farRootSet)
                    {
                        farRootField.SetValue(e.far, e.farRoot);
                        changed = true;
                    }

                    if (FarSweep(e.far) == e.farSweepSet)
                    {
                        farSweepField.SetValue(e.far, e.farSweep);
                        changed = true;
                    }

                    if (changed)
                    {
                        farStart.Invoke(e.far, null);
                        farChanged = true;
                    }
                }
            }

            entries.Clear();
            leftOut.Clear();
            applied = 0f;
            return farChanged;
        }

        private static bool IsMovable(WingProcedural wp)
        {
            return wp != null && wp.CanVarySweep && wp.SweepEnabled;
        }

        private bool StillValid()
        {
            for (int i = 0; i < entries.Count; ++i)
            {
                // Entry 0 is the wing itself, and movable by definition.
                if (entries[i].part == null || (i > 0 && IsMovable(entries[i].wing)))
                {
                    return false;
                }
            }

            for (int i = 0; i < leftOut.Count; ++i)
            {
                if (leftOut[i] == null || !IsMovable(leftOut[i]))
                {
                    return false;
                }
            }

            return true;
        }

        private void Capture(Part wing)
        {
            entries.Clear();
            leftOut.Clear();
            CaptureTree(wing, true);
        }

        private void CaptureTree(Part p, bool isPivot)
        {
            WingProcedural wp = p.Modules.GetModule<WingProcedural>();
            if (!isPivot && IsMovable(wp))
            {
                leftOut.Add(wp);
                return;
            }

            Entry e = new Entry { part = p, wing = wp, com = p.CoMOffset, col = p.CoLOffset };
            e.comSet = e.com;
            e.colSet = e.col;
            e.far = FarWingOf(p);
            if (e.far != null)
            {
                e.farRoot = e.farRootSet = FarRoot(e.far);
                e.farSweep = e.farSweepSet = FarSweep(e.far);
            }

            entries.Add(e);
            for (int i = 0; i < p.children.Count; ++i)
            {
                if (p.children[i] != null)
                {
                    CaptureTree(p.children[i], false);
                }
            }
        }

        // Whatever no longer holds the value written here was set by someone else, and is the
        // neutral from now on.
        private static bool TakeExternalChanges(Entry e)
        {
            bool changed = false;
            if (e.part.CoMOffset != e.comSet)
            {
                e.com = e.part.CoMOffset;
                changed = true;
            }

            if (e.part.CoLOffset != e.colSet)
            {
                e.col = e.part.CoLOffset;
                changed = true;
            }

            if (e.far != null)
            {
                Vector3 root = FarRoot(e.far);
                if (root != e.farRootSet)
                {
                    e.farRoot = root;
                    changed = true;
                }

                double sweep = FarSweep(e.far);
                if (sweep != e.farSweepSet)
                {
                    e.farSweep = sweep;
                    changed = true;
                }
            }

            return changed;
        }

        private static Vector3 Turn(Transform pivot, Quaternion rotation, Vector3 world)
        {
            return pivot.TransformPoint(rotation * pivot.InverseTransformPoint(world));
        }

        /// <summary>
        /// FAR places a wing's lift at a centroid it works out from the root mid-chord offset and
        /// the planform, in the part frame. Turn the planform's sweep with the wing, then shift the
        /// root offset by whatever it takes to land that centroid on the neutral one turned about
        /// the pivot - so the point is exact whatever the wing's own sweep, and the planform terms
        /// (span, taper) are left alone. Only the centroid is recomputed here; FinishFar does the
        /// rest once the slider stops.
        /// </summary>
        private static void PoseFar(Entry e, Transform pivot, Quaternion rotation, bool isWing, float planformSweep)
        {
            e.farSweepSet = e.farSweep + planformSweep;
            if (farCentroidField != null)
            {
                farRootField.SetValue(e.far, e.farRoot);
                farSweepField.SetValue(e.far, e.farSweep);
                farInit.Invoke(e.far, null);
                Transform t = e.part.partTransform;
                Vector3 neutral = (Vector3)(Vector3d)farCentroidField.GetValue(e.far);
                Vector3 target = t.InverseTransformDirection(Turn(pivot, rotation, t.TransformDirection(neutral) + t.position) - t.position);

                farSweepField.SetValue(e.far, e.farSweepSet);
                if (planformSweep != 0f)
                {
                    farInit.Invoke(e.far, null);
                }

                Vector3 swept = (Vector3)(Vector3d)farCentroidField.GetValue(e.far);
                e.farRootSet = e.farRoot + (target - swept);
                farRootField.SetValue(e.far, e.farRootSet);
                farInit.Invoke(e.far, null);
            }
            else
            {
                // FAR without the centroid field: turn the wing's own root offset, and nothing more.
                e.farRootSet = isWing ? rotation * e.farRoot : e.farRoot;
                farRootField.SetValue(e.far, e.farRootSet);
                farSweepField.SetValue(e.far, e.farSweepSet);
            }

            e.farPending = true;
        }

        // FAR is not referenced at build time, so its wing model is reached by reflection, bound
        // once from the first one found.
        private static bool farBound;
        private static FieldInfo farRootField;     // Vector3 rootMidChordOffsetFromOrig
        private static FieldInfo farSweepField;    // double MidChordSweep
        private static FieldInfo farCentroidField; // Vector3d localWingCentroid (private)
        private static MethodInfo farInit;         // MathAndFunctionInitialization: recomputes the centroid
        private static MethodInfo farStart;        // StartInitialization: that, plus wing interactions

        private static PartModule FarWingOf(Part p)
        {
            if (!WingProcedural.assemblyFARUsed)
            {
                return null;
            }

            for (int i = 0; i < p.Modules.Count; ++i)
            {
                PartModule m = p.Modules[i];
                for (Type t = m.GetType(); t != null && t != typeof(PartModule); t = t.BaseType)
                {
                    if (t.Name == "FARWingAerodynamicModel")
                    {
                        return Bind(t) ? m : null;
                    }
                }
            }

            return null;
        }

        private static bool Bind(Type t)
        {
            if (!farBound)
            {
                farBound = true;
                const BindingFlags inst = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
                farRootField = t.GetField("rootMidChordOffsetFromOrig", inst);
                farSweepField = t.GetField("MidChordSweep", inst);
                farCentroidField = t.GetField("localWingCentroid", inst);
                farInit = t.GetMethod("MathAndFunctionInitialization", inst, null, Type.EmptyTypes, null);
                farStart = t.GetMethod("StartInitialization", inst, null, Type.EmptyTypes, null);

                if (farRootField?.FieldType != typeof(Vector3) || farSweepField?.FieldType != typeof(double) || farStart == null)
                {
                    Debug.Log("B9 PWings - FAR wing model fields not found, the sweep preview leaves FAR's center of lift alone");
                    farRootField = null;
                }

                if (farCentroidField?.FieldType != typeof(Vector3d) || farInit == null)
                {
                    farCentroidField = null;
                }
            }

            return farRootField != null;
        }

        private static Vector3 FarRoot(PartModule far) => (Vector3)farRootField.GetValue(far);

        private static double FarSweep(PartModule far) => (double)farSweepField.GetValue(far);
    }
}
