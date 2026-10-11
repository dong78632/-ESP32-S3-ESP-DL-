# Local ESP-DL component (optional)

The default main/idf_component.yml downloads the exact ESP-DL v2.0 commit.
If you prefer a local component, clone the complete repository here:

```sh
git clone --branch v2.0 --depth 1 https://github.com/espressif/esp-dl.git components/esp-dl
```

Then remove the `esp-dl` Git dependency from `main/idf_component.yml` to avoid
maintaining two dependency sources. Keep `REQUIRES esp-dl` in main/CMakeLists.txt.

Expected directory structure:

```text
components/esp-dl/CMakeLists.txt
components/esp-dl/idf_component.yml
components/esp-dl/include/model_zoo/human_face_detect_msr01.hpp
components/esp-dl/include/model_zoo/human_face_detect_mnp01.hpp
components/esp-dl/lib/esp32s3/libhuman_face_detect.a
components/esp-dl/lib/esp32s3/libdl.a
```

Copying only the headers is insufficient. The complete component publishes its
include directories and links the target-specific precompiled model libraries.
The classic v2.0 component requires ESP-IDF 5.0.x, not ESP-DL v3.x APIs.
