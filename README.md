# Todo

- [ ] camera module
- [ ] path planning
- [ ] grippers
- [ ] factory functions

# Notes

By default, each revolute joint and prismatic joint is motorized using a velocity motor. We can perform torque control by setting the maxforce of each joint motor to 0.

```python
maxForce = 0
mode = p.VELOCITY_CONTROL
p.setJointMotorControl2(objUid, jointIndex, controlMode=mode, force=maxForce)
```
