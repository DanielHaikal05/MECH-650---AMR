from pathlib import Path

base_dir = Path.home() / "AMR-Project/src/amr_town_world/models"

for i in range(1, 11):
    model_name = f"qr{i}"
    sdf_name = f"qr_code{i}.sdf"

    model_dir = base_dir / model_name
    model_dir.mkdir(parents=True, exist_ok=True)

    config_text = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.7">{sdf_name}</sdf>

  <author>
    <name>Suhaib</name>
  </author>

  <description>
    QR landmark {i}
  </description>
</model>
"""

    config_path = model_dir / "model.config"
    config_path.write_text(config_text)

    print(f"Created: {config_path}")