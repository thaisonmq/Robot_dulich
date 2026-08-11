#version 120

// RViz Humble's stock occupancy-grid shader combines sampler2D and sampler1D.
// Some Mesa/OGRE combinations link both samplers to texture unit zero before
// Ogre applies the material parameters, leaving the viewport magenta.  This
// project preset renders the standard map/costmap palettes directly from the
// occupancy value and therefore needs only the map's sampler2D.

varying vec2 UV;
uniform sampler2D eight_bit_image;
uniform float alpha;

void main()
{
  float raw_value = floor(texture2D(eight_bit_image, UV).x * 255.0 + 0.5);
  vec3 color;
  float output_alpha = alpha;

  // Project map displays use alpha > 0.5. Costmap displays use alpha <= 0.5.
  if (alpha > 0.5) {
    if (raw_value <= 100.0) {
      float gray = 1.0 - raw_value / 100.0;
      color = vec3(gray);
    } else if (raw_value <= 127.0) {
      color = vec3(0.0, 1.0, 0.0);
    } else if (raw_value <= 254.0) {
      color = vec3(1.0, (raw_value - 128.0) / 126.0, 0.0);
    } else {
      color = vec3(112.0 / 255.0, 137.0 / 255.0, 134.0 / 255.0);
    }
  } else {
    if (raw_value < 0.5) {
      color = vec3(0.0);
      output_alpha = 0.0;
    } else if (raw_value <= 98.0) {
      float red = raw_value / 100.0;
      color = vec3(red, 0.0, 1.0 - red);
    } else if (raw_value < 99.5) {
      color = vec3(0.0, 1.0, 1.0);
    } else if (raw_value < 100.5) {
      color = vec3(1.0, 0.0, 1.0);
    } else if (raw_value <= 127.0) {
      color = vec3(0.0, 1.0, 0.0);
    } else if (raw_value <= 254.0) {
      color = vec3(1.0, (raw_value - 128.0) / 126.0, 0.0);
    } else {
      color = vec3(112.0 / 255.0, 137.0 / 255.0, 134.0 / 255.0);
    }
  }

  gl_FragColor = vec4(color, output_alpha);
}
