# Integrated Explorer V3 - Design Document

## 🔴 Problem Analysis: Why V2 Fails

### Critical Issues in V2

1. **Robot Position Not Updating Correctly**
   - `_execution_step()` calls `robot.move_to()` but might fail silently
   - No verification that robot actually moved
   - Trajectory array might not be populated correctly

2. **Goal Not Changing Between Iterations**
   - `previous_best_branch` reuse causes stale goals
   - RRT builds tree from current position but reuses old branch with wrong parent indices
   - Branch reuse disabled (set to None) but goals still static in some cases

3. **State Management Issues**
   - Robot state (`pose`, `trajectory`, `observations`) scattered across code
   - No single source of truth for robot position
   - Unclear when state updates happen

4. **RRT Planning Issues**
   - `root_pose=self.robot.pose` might use stale pose
   - Tree building doesn't verify root position
   - No logging to confirm tree starts from current position

5. **Execution Flow Confusion**
   - `_execution_step()` returns tuple but some callers don't unpack
   - `n_nodes` calculation might be wrong
   - Minimum movement check might skip iterations unexpectedly

## 🎯 V3 Design Goals

### 1. **Explicit State Management**
- Clear, immutable state snapshots
- Explicit state transitions
- Every iteration returns new state

### 2. **Verified Robot Movement**
- Mandatory movement verification
- Logged position before/after
- Trajectory recorded immediately after movement

### 3. **Fresh Planning Each Iteration**
- RRT builds from verified current position
- No branch reuse (until properly implemented)
- New goals every iteration based on current state

### 4. **Robust Error Handling**
- Check every assumption
- Fail loudly if something is wrong
- Comprehensive logging

### 5. **Testable Design**
- Pure functions where possible
- Dependency injection
- Easy to verify state changes

## 🏗️ V3 Architecture

### Core Principles

```python
class IterationState:
    """Immutable state snapshot for one iteration."""
    iteration: int
    robot_pose: Pose2D
    robot_trajectory: List[Tuple[float, float]]
    observations: List[Tuple[np.ndarray, float]]
    estimated_sources: List[np.ndarray]
    rfc: float
    best_rfc: float

class IterationResult:
    """Result of executing one iteration."""
    state_before: IterationState
    state_after: IterationState

    # Detailed step results
    estimation_result: EstimationStepResult
    planning_result: PlanningStepResult
    execution_result: ExecutionStepResult

    # Verification
    robot_moved: bool
    movement_distance: float
    goal_changed: bool
```

### Execution Flow

```
ITERATION N:

1. CAPTURE INITIAL STATE
   - Record robot.pose
   - Record len(trajectory)
   - Record len(observations)

2. ESTIMATION STEP
   - Run ADE-PSPF with current observations
   - Get estimated sources
   - Calculate RFC

3. PLANNING STEP (RRT)
   - VERIFY: tree starts from robot.pose
   - Build tree from CURRENT position
   - NO branch reuse
   - Select best branch
   - LOG: goal = best_branch[-1].pose

4. EXECUTION STEP
   - LOG: moving from (x1, y1) to (x2, y2)
   - EXECUTE: robot.move_to(...)
   - VERIFY: robot.pose changed
   - VERIFY: trajectory appended
   - VERIFY: observations made

5. CAPTURE FINAL STATE
   - Record new robot.pose
   - Record new len(trajectory)
   - Record new len(observations)

6. VERIFICATION
   - ASSERT: state_after.robot_pose != state_before.robot_pose
   - ASSERT: len(state_after.trajectory) > len(state_before.trajectory)
   - LOG: "Robot moved from X to Y, distance = D"
```

## 🔧 Key Implementation Changes

### 1. RobotState Class Enhancement

```python
class RobotState:
    def move_to(self, x: float, y: float, theta: float) -> bool:
        """
        Move robot and verify state change.

        Returns:
            True if move successful, False otherwise
        """
        # Record old position
        old_x, old_y = self.pose.x, self.pose.y

        # Add to trajectory BEFORE updating pose
        self.trajectory.append((old_x, old_y))

        # Update pose
        self.pose = Pose2D(x=x, y=y, theta=theta)

        # Verify movement
        moved = (old_x != x) or (old_y != y)

        if not moved:
            print(f"⚠️ WARNING: Robot didn't move! Still at ({old_x}, {old_y})")
        else:
            dist = np.sqrt((x - old_x)**2 + (y - old_y)**2)
            print(f"✓ Robot moved: ({old_x:.1f}, {old_y:.1f}) → ({x:.1f}, {y:.1f}), dist={dist:.1f}px")

        return moved
```

### 2. Planning Step with Verification

```python
def _planning_step(self, sources: List[np.ndarray]) -> PlanningStepResult:
    """
    Plan path using RRT with strict verification.
    """
    # Get CURRENT robot position
    current_pose = self.robot.pose

    print(f"[PLANNING] Starting RRT from ({current_pose.x:.1f}, {current_pose.y:.1f})")

    # Build tree - NO previous branch reuse
    nodes, leaves = self.planner.build_tree(
        root_pose=current_pose,  # Use CURRENT position
        sources=np.array(sources),
        observation_positions=...,
        observation_intensities=...,
        nearest_intensity_lookup=...,
        previous_best_branch=None  # ALWAYS None
    )

    # Verify tree root
    assert nodes[0].pose.x == current_pose.x, "Tree root mismatch!"
    assert nodes[0].pose.y == current_pose.y, "Tree root mismatch!"

    # Select best branch
    best_branch, best_leaf_idx = self.planner.select_best_branch(nodes, leaves)

    # Get goal
    goal_pose = best_branch[-1].pose if best_branch else current_pose

    print(f"[PLANNING] Goal selected: ({goal_pose.x:.1f}, {goal_pose.y:.1f})")

    return PlanningStepResult(
        nodes=nodes,
        leaves=leaves,
        best_branch=best_branch,
        goal_pose=goal_pose,
        tree_root_pose=current_pose
    )
```

### 3. Execution Step with Mandatory Verification

```python
def _execution_step(self, best_branch: List[RRTNode]) -> ExecutionStepResult:
    """
    Execute path with mandatory verification.
    """
    if len(best_branch) < 2:
        return ExecutionStepResult(moved=False, distance=0.0)

    # Record state BEFORE movement
    pose_before = Pose2D(
        x=self.robot.pose.x,
        y=self.robot.pose.y,
        theta=self.robot.pose.theta
    )
    trajectory_len_before = len(self.robot.trajectory)

    print(f"[EXECUTION] Before: ({pose_before.x:.1f}, {pose_before.y:.1f}), "
          f"trajectory_len={trajectory_len_before}")

    # Execute FIRST EDGE only
    target_node = best_branch[1]

    print(f"[EXECUTION] Moving to: ({target_node.pose.x:.1f}, {target_node.pose.y:.1f})")

    # MOVE
    moved = self.robot.move_to(
        target_node.pose.x,
        target_node.pose.y,
        target_node.pose.theta
    )

    # Make observation at new position
    if moved:
        self._make_observation(target_node.pose.x, target_node.pose.y)

    # Record state AFTER movement
    pose_after = self.robot.pose
    trajectory_len_after = len(self.robot.trajectory)

    print(f"[EXECUTION] After: ({pose_after.x:.1f}, {pose_after.y:.1f}), "
          f"trajectory_len={trajectory_len_after}")

    # VERIFY
    distance = np.sqrt(
        (pose_after.x - pose_before.x)**2 +
        (pose_after.y - pose_before.y)**2
    )

    trajectory_grew = trajectory_len_after > trajectory_len_before

    if not moved:
        print("❌ ERROR: Robot.move_to() returned False!")

    if distance < 0.1:
        print(f"❌ ERROR: Robot didn't actually move! Distance = {distance:.4f}")

    if not trajectory_grew:
        print(f"❌ ERROR: Trajectory didn't grow! {trajectory_len_before} → {trajectory_len_after}")

    success = moved and distance > 0.1 and trajectory_grew

    return ExecutionStepResult(
        moved=success,
        distance=distance,
        pose_before=pose_before,
        pose_after=pose_after,
        observations_made=1 if moved else 0
    )
```

## 📋 Implementation Checklist

### Phase 1: Core Structure
- [ ] Create `integrated_explorer_v3.py`
- [ ] Define `IterationState` dataclass
- [ ] Define `IterationResult` dataclass
- [ ] Define step result dataclasses (Estimation, Planning, Execution)
- [ ] Implement enhanced `RobotState` with verification

### Phase 2: Core Methods
- [ ] Implement `_capture_state()` - snapshot current state
- [ ] Implement `_estimation_step()` - with logging
- [ ] Implement `_planning_step()` - with verification
- [ ] Implement `_execution_step()` - with mandatory checks
- [ ] Implement `_verify_iteration()` - post-iteration checks

### Phase 3: Main Loop
- [ ] Implement `run_exploration()` with clear flow
- [ ] Add comprehensive logging at each step
- [ ] Add state verification after each iteration
- [ ] Handle errors gracefully

### Phase 4: Integration
- [ ] Make V3 compatible with explorer_controller
- [ ] Update explorer_controller to use V3
- [ ] Add debug mode with extra verbose logging

### Phase 5: Testing
- [ ] Create `test_explorer_v3_basic.py`
- [ ] Test robot movement verification
- [ ] Test goal change verification
- [ ] Test state consistency
- [ ] Run full app integration test

## 🎯 Success Criteria

V3 will be considered successful when:

1. ✅ Robot position updates every iteration (verified with assertions)
2. ✅ Goal changes every iteration (unless RRT happens to select same point)
3. ✅ Trajectory grows by 1 point per iteration
4. ✅ Observations increase by 1 per iteration
5. ✅ All state changes are logged and verifiable
6. ✅ App visualization shows robot moving
7. ✅ App visualization shows goal changing

## 🚨 Anti-Patterns to Avoid

1. ❌ Silent failures (robot.move_to() without checking return)
2. ❌ Unverified state changes (assume robot moved without checking)
3. ❌ Branch reuse without proper parent index remapping
4. ❌ Tuple unpacking mismatches
5. ❌ Stale robot pose in RRT planning
6. ❌ Missing logging at critical points

## 📝 Migration Notes

### From V2 to V3

**Breaking Changes:**
- Method signatures changed for clarity
- All step methods return result objects
- State is captured explicitly before/after each step

**Compatibility:**
- V3 will be drop-in replacement for V2 in explorer_controller
- Same external interface
- Same configuration options

**Testing:**
- V2 and V3 can coexist
- Use config flag to select version
- Compare results side-by-side
