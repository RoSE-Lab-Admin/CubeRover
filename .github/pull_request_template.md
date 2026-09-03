## 🛑 REQUIRED: Check your Base Branch!
**Are you submitting a software change?** Please ensure the "base" branch is set to **`dev`**, NOT `main`. 
*All software changes must be validated on physical hardware via the `dev` branch before being merged into `main`.*

---

### ✅ PR Checklist
- [ ] I have set the base branch to **`dev`**.
- [ ] I have verified that all software unit tests pass locally using:
    ```
    colcon test --event-handlers console_cohesion+ --ctest-args -R "software"
    ```
- [ ] I have verified that my changes do not break the existing hardware-related logic.