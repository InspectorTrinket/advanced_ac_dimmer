# Publishing to GitHub — Step by Step

## Prerequisites
- Git installed on your machine
- GitHub account
- GitHub CLI (`gh`) optional but recommended

---

## 1. Create the repository on GitHub

Go to https://github.com/new and fill in:
- **Repository name:** `advanced_ac_dimmer`
- **Description:** `ESPHome external component for AC phase-angle dimming with configurable ZCD method, multi-cycle kickstart, RMS correction control, and flat zone elimination`
- **Visibility:** Public (required for ESPHome external_components to pull it)
- **Initialize with README:** No (you already have one)

Click **Create repository**.

---

## 2. Initialize and push from your machine

Open a terminal in the folder where you extracted the zip, then:

```bash
cd advanced_ac_dimmer

git init
git add .
git commit -m "Initial release: advanced_ac_dimmer ESPHome component"

git remote add origin https://github.com/YOUR_USERNAME/advanced_ac_dimmer.git
git branch -M main
git push -u origin main
```

Replace `YOUR_USERNAME` with your actual GitHub username.

---

## 3. Create a release tag (recommended)

ESPHome can reference a specific tag for reproducible builds:

```bash
git tag v1.0.0
git push origin v1.0.0
```

On GitHub, go to **Releases → Draft a new release**, select tag `v1.0.0`, and publish.

---

## 4. Use in ESPHome

### Latest from main branch:
```yaml
external_components:
  - source: github://YOUR_USERNAME/advanced_ac_dimmer
    components: [advanced_ac_dimmer]
```

### Pinned to a specific release (recommended for production):
```yaml
external_components:
  - source: github://YOUR_USERNAME/advanced_ac_dimmer@v1.0.0
    components: [advanced_ac_dimmer]
```

---

## 5. Update the component later

After making changes:

```bash
git add .
git commit -m "Description of what changed"
git push

# If it's a new release:
git tag v1.1.0
git push origin v1.1.0
```

---

## Repository structure

```
advanced_ac_dimmer/
├── README.md                          ← Documentation
├── PUBLISHING.md                      ← This file
└── components/
    └── advanced_ac_dimmer/
        ├── __init__.py                ← Empty, required by ESPHome
        ├── output.py                  ← YAML schema and code generation
        ├── advanced_ac_dimmer.h       ← C++ class declarations
        ├── advanced_ac_dimmer.cpp     ← C++ implementation
        ├── hw_timer_esp_idf.h         ← ESP32 hardware timer abstraction
        └── hw_timer_esp_idf.cpp       ← ESP32 hardware timer implementation
```
