uniform float uVertexScale;
uniform float uTime;
uniform float uScaleX;
uniform float uScaleY;

attribute vec2 aPosition;
attribute vec3 aColor;
attribute vec2 aTexCoord0, aTexCoord1;


varying vec3 vColor;
varying vec2 vTexCoord0, vTexCoord1;

void main() {
  float lerper = sin(uTime/8000.0)+1.0;
  gl_Position = vec4(aPosition.x * uVertexScale*uScaleX * lerper, aPosition.y*uScaleY, 0,1);
  vColor = aColor;
  vTexCoord0 = aTexCoord0;
  vTexCoord1 = aTexCoord1;
}
