#ifndef DIRECTIONAL_LIGHT_COUNT
# define DIRECTIONAL_LIGHT_COUNT 0
#endif

#ifndef SPOT_LIGHT_COUNT
# define SPOT_LIGHT_COUNT 0
#endif

#ifndef POINT_LIGHT_COUNT
# define POINT_LIGHT_COUNT 0
#endif

#if (DIRECTIONAL_LIGHT_COUNT > 0) || (POINT_LIGHT_COUNT > 0) || (SPOT_LIGHT_COUNT > 0)
# define LIGHTING
#endif

///////////////////////////////////////////////////////////
// Attributes
attribute vec4 a_position;

attribute vec2 a_texCoord;

#if defined(LIGHTING)
  attribute vec3 a_normal;
#endif

attribute vec3 a_color;
varying vec3 v_color;

///////////////////////////////////////////////////////////
// Uniforms
uniform mat4 u_worldViewProjectionMatrix;

#if defined(LIGHTING)
  uniform mat4 u_inverseTransposeWorldViewMatrix;

  #if defined(SPECULAR) || (POINT_LIGHT_COUNT > 0) || (SPOT_LIGHT_COUNT > 0)
    uniform mat4 u_worldViewMatrix;
  #endif

  #if (POINT_LIGHT_COUNT > 0)
    uniform vec3 u_pointLightPosition[POINT_LIGHT_COUNT];
  #endif

  #if (SPOT_LIGHT_COUNT > 0)
    uniform vec3 u_spotLightPosition[SPOT_LIGHT_COUNT];
    #if defined(BUMPED)
      uniform vec3 u_spotLightDirection[SPOT_LIGHT_COUNT];
    #endif
  #endif

  #if defined(SPECULAR)
    uniform vec3 u_cameraPosition;
  #endif
#endif // LIGHTING

#if defined(TEXTURE_REPEAT)
  uniform vec2 u_textureRepeat;
#endif

#if defined(TEXTURE_OFFSET)
  uniform vec2 u_textureOffset;
#endif

#if defined(CLIP_PLANE)
  uniform mat4 u_worldMatrix;
  uniform vec4 u_clipPlane;
#endif

///////////////////////////////////////////////////////////
// Varyings
varying vec2 v_texCoord;

#if defined(LIGHTING)
  #if (POINT_LIGHT_COUNT > 0)
    varying vec3 v_vertexToPointLightDirection[POINT_LIGHT_COUNT];
  #endif

  #if (SPOT_LIGHT_COUNT > 0)
    varying vec3 v_vertexToSpotLightDirection[SPOT_LIGHT_COUNT];
    #if defined(BUMPED)
      varying vec3 v_spotLightDirection[SPOT_LIGHT_COUNT];
    #endif
  #endif

  #if defined(SPECULAR)
    varying vec3 v_cameraDirection;
  #endif

  #include "lighting.vert"
#endif // LIGHTING

#if defined(CLIP_PLANE)
  varying float v_clipDistance;
#endif

void main()
{
  vec4 position = a_position;
  gl_Position = u_worldViewProjectionMatrix * position;

  #if defined(LIGHTING)
    vec3 normal = a_normal;
    // Transform the normal, tangent and binormals to view space.
    mat3 inverseTransposeWorldViewMatrix = mat3(u_inverseTransposeWorldViewMatrix[0].xyz, u_inverseTransposeWorldViewMatrix[1].xyz, u_inverseTransposeWorldViewMatrix[2].xyz);
    vec3 normalVector = normalize(inverseTransposeWorldViewMatrix * normal);

    #if defined(BUMPED)
      vec3 tangent = getTangent();
      vec3 binormal = getBinormal();
      vec3 tangentVector  = normalize(inverseTransposeWorldViewMatrix * tangent);
      vec3 binormalVector = normalize(inverseTransposeWorldViewMatrix * binormal);
      mat3 tangentSpaceTransformMatrix = mat3(tangentVector.x, binormalVector.x, normalVector.x, tangentVector.y, binormalVector.y, normalVector.y, tangentVector.z, binormalVector.z, normalVector.z);
      applyLight(position, tangentSpaceTransformMatrix);
    #else
      v_normalVector = normalVector;
      applyLight(position);
    #endif
  #endif // LIGHTING

  v_texCoord = a_texCoord;

  #if defined(TEXTURE_REPEAT)
    v_texCoord *= u_textureRepeat;
  #endif

  #if defined(TEXTURE_OFFSET)
    v_texCoord += u_textureOffset;
  #endif

  #if defined(CLIP_PLANE)
    v_clipDistance = dot(u_worldMatrix * position, u_clipPlane);
  #endif

  v_color = a_color;
}
