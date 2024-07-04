#define STBI_ONLY_PNG
#define STB_IMAGE_IMPLEMENTATION

#include <stdexcept>
#include <glsl/stb_image.h>
#include "shader.h"


namespace shader {


  // Common uniform block.
  const char* uniformBlock = R"glsl(

    #version 440 core
  
    layout (std140, binding = 1) uniform UniformBlock
    {
      mat4 V;            // View
      mat4 P;            // Projection
      mat4 MV;           // ModelView
      mat4 MVP;          // ModelViewProjection
      mat4 ITMV;         // Inverse-transpose model view
      mat4 Ortho;        // Ortho
    };

    struct instance_proxy {
      mat4 B;
      float speed;
      float tex;
      float alpha;
      float _[3];      // padding
    };

  )glsl";


  const char* instanceVertex = R"glsl(

    layout (location = 0) uniform float scale = 1.0;
    layout (location = 1) uniform float ambient = 0.3;

    layout (location = 0) in vec4 Vertex;
    layout (location = 1) in vec4 Normal;
    layout (location = 2) in vec2 TexCoord;
    layout (location = 3) in vec4 forward4;
    layout (location = 4) in vec4 side4;
    layout (location = 5) in vec4 pos4;
    layout (location = 6) in vec4 sta;    // [speed, tex, alpha]

    smooth out vec2 vTex;
    flat out float vSpectrum;
    flat out float vAlpha;
    smooth out float vShade;

    void main()
    {
      vTex = TexCoord;
      vSpectrum = sta.y;
      vAlpha = sta.z;

      // construct model matrix
      vec4 v = vec4(scale * Vertex.xyz, 1.0);
      mat4 M = mat4( forward4,
                     side4,
                     vec4(cross(vec3(forward4), vec3(side4)), 0.0),
                     pos4);

      // add a simple headlight
      vec4 normal = MV * M * vec4(Normal.xyz, 0);   // normal in view space
      float ds = abs(dot(normal, vec4(0,0,1,0)));   // (0,0,1,0): eye in view space
      vShade = ambient + (1.0 - ambient) * ds;

      gl_Position = MVP * (M * v);
    }

  )glsl";


  const char* instanceFragment = R"glsl(

    layout (location = 4) uniform float texMix = 0.0;
    layout (binding = 1) uniform sampler2D Texture;
    layout (binding = 2) uniform sampler2D Spectrum;

    smooth in vec2 vTex;
    flat in float vSpectrum;
    flat in float vAlpha;
    smooth in float vShade;
    
    layout (location = 0) out vec4 FragColor;

    void main()
    {
      vec4 color = texture(Texture, vTex);
      float fade = length(color.rgb);      
      float tm = texMix * ((vSpectrum < 0.0) ? 0.0 : 1.0);
      color = mix(color, texture(Spectrum, vec2(vSpectrum, 0.5)), tm);
      FragColor.rgb = vShade * color.rgb;
      FragColor.a = (1.0 - fade);
    }

  )glsl";


  const char* immVertex = R"glsl(

    layout (location = 0) in vec3 Vertex;
    layout (location = 1) in vec4 Color;
    layout (location = 2) in float Tex;

    smooth out vec4 vColor;
    flat out float vTex;
  
    void main(void) 
    {            
      vColor = Color;
      vTex = Tex;
      gl_Position = MVP * vec4(Vertex, 1.0);
    }

  )glsl";


  const char* immVertexInst  = R"glsl(

    layout (location = 0) in vec3 Vertex;
    layout (location = 1) in vec4 Color;
    layout (location = 2) in float Tex;
    
    layout (location = 0) uniform vec3 dxy;

    smooth out vec4 vColor;
    flat out float vTex;
  
    void main(void) 
    {            
      float dx = dxy[gl_InstanceID % 3];
      float dy = dxy[gl_InstanceID / 3];
      vColor = Color;
      vTex = Tex;
      vec4 vertex = vec4(Vertex.x + dx, Vertex.y + dy, 0.0, 1.0);
      gl_Position = MVP * vertex;
    }

  )glsl";


  const char* immFragment = R"glsl(

    smooth in vec4 vColor;
    flat in float vTex;
    out vec4 FragColor;

    void main(void) 
    {            
      if (vTex == -100.0) discard;
      FragColor = vColor;
    }

  )glsl";


  const char* ribbonVertex = R"glsl(
    layout (location = 0) uniform int N = 1;      // instances
    layout (location = 1) uniform float S = 10;   // vertices per instance
    layout (location = 2) uniform vec2 fade;      // fade0, fade1

    layout (std430, binding=1) buffer proxy
    { 
      instance_proxy inst[];
    };

    flat out vec3 vVertex;
    flat out vec3 vSide;
    flat out float vSpectrum;
    flat out float vAlpha;

    void main()
    {
      int i = gl_InstanceID + N * gl_VertexID;
      vVertex = inst[i].B[3].xyz;
      vSide = inst[i].B[2].xyz;
      vSpectrum = inst[i].tex;
      vec2 alpha = vec2(1) - fade;
      float as = alpha.y * (1.0 - float(gl_VertexID) / S);
      vAlpha =  alpha.x * as;
    }

  )glsl";


  const char* ribbonGeo = R"glsl(
    layout(lines_adjacency) in;
    layout(triangle_strip, max_vertices=4) out;

    layout (binding = 2) uniform sampler2D Spectrum;
    layout (location = 3) uniform float ambient = 0.5;
    layout (location = 4) uniform float hw = 0.1;

    flat in vec3 vVertex[4];
    flat in vec3 vSide[4]; 
    flat in float vSpectrum[4];
    flat in float vAlpha[4];

    smooth out vec4 gColor;
    smooth out float gAlpha;
    smooth out float gShade;

    const vec4 eye = vec4(0.0, 0.0, 1.0, 0.0);

    float shade(vec3 normal)
    {
      // simple hedlight
      float ds = abs(dot(MV * vec4(normal, 0.0), eye));
      return (1.0 - ambient) * ds + ambient;
    }

    void Emit(vec3 v, float alpha)
    {
      gAlpha = alpha;
      gl_Position = MVP * vec4(v, 1.0);
      EmitVertex();
    }

    void main()
    {
      vec3 tangent01 = normalize(vVertex[1]-vVertex[0]);
      vec3 tangent21 = normalize(vVertex[2]-vVertex[1]);
      vec3 tangent23 = normalize(vVertex[3]-vVertex[2]);

      vec3 up0 = cross(vSide[0], tangent01);
      vec3 up1 = cross(vSide[1], tangent21);
      vec3 up2 = cross(vSide[2], tangent23);
    
      float shade0 = shade(up0);
      float shade1 = shade(up1);
      float shade2 = shade(up2);
    
      vec4 color0 = texture(Spectrum, vec2(vSpectrum[0], 0.5));
      vec4 color1 = texture(Spectrum, vec2(vSpectrum[1], 0.5));
      vec4 color2 = texture(Spectrum, vec2(vSpectrum[2], 0.5));

      gColor = mix(color0, color1, 0.5);
      gShade = mix(shade0, shade1, 0.5);
      Emit(vVertex[1] - hw * vSide[1], vAlpha[1]);
      Emit(vVertex[1] + hw * vSide[1], vAlpha[1]);
    
      gColor = mix(color1, color2, 0.5);
      gShade = mix(shade1, shade2, 0.5);
      Emit(vVertex[2] - hw * vSide[2], vAlpha[2]);
      Emit(vVertex[2] + hw * vSide[2], vAlpha[2]);
    }
  )glsl";


  const char* ribbonFragment = R"glsl(
    smooth in vec4 gColor;
    smooth in float gShade;
    smooth in float gAlpha;
    
    layout (location = 0) out vec4 FragColor;

    void main()
    {
      FragColor.rgb = (gShade * gColor).rgb;
      FragColor.a = gAlpha;
    }

  )glsl";


  const char* skyboxVertex = R"glsl(

    const vec3[12 * 3] vertices = vec3[12 * 3](
      vec3(1,  1,  1),   vec3(1, -1, -1),   vec3(1, -1,  1),
      vec3(1, -1, -1),   vec3(1,  1,  1),   vec3(1,  1, -1),
      vec3(-1,  1, -1),  vec3(-1, -1,  1),  vec3(-1, -1, -1),
      vec3(-1, -1,  1),  vec3(-1,  1, -1),  vec3(-1,  1,  1),
      vec3(1, -1, -1),   vec3(-1,  1, -1),  vec3(-1, -1, -1),
      vec3(-1,  1, -1),  vec3(1, -1, -1),   vec3(1,  1, -1),
      vec3(1,  1,  1),   vec3(-1, -1,  1),  vec3(-1,  1,  1),
      vec3(-1, -1,  1),  vec3(1,  1,  1),   vec3(1, -1,  1),
      vec3(1,  1,  1),   vec3(-1,  1, -1),  vec3(1,  1, -1),
      vec3(-1,  1, -1),  vec3(1,  1,  1),   vec3(-1,  1,  1),
      vec3(-1, -1,  1),  vec3(1, -1, -1),   vec3(-1, -1, -1),
      vec3(1, -1, -1),   vec3(-1, -1,  1),  vec3(1, -1,  1)
    );

    out vec3 vViewDir;

    void main(void)
    {
      gl_Position = P * vec4(mat3(MV) * vertices[gl_VertexID], 1.0);
      vViewDir = vertices[gl_VertexID];
    }

  )glsl";


  const char* skyboxFragment = R"glsl(

    layout (location = 0) uniform float colorFact = 1.0;
    layout (binding = 3) uniform samplerCube cubeTex;
 
    in vec3 vViewDir;
    out vec4 FragColor;

    void main(void)
    {
      FragColor.rgb = colorFact * texture(cubeTex, vViewDir).rgb;
      FragColor.a = 1.0;
      gl_FragDepth = 1.0;
    }

  )glsl";


  namespace {

    GLuint ShaderFromLiteral(const char* shaderSource, GLenum shaderType) noexcept
    {
      const auto sh = glCreateShader(shaderType);
      glShaderSource(sh, 1, &shaderSource, 0);
      glCompileShader(sh);
      return sh;
    }

  }


  GLuint ProgFromLiterals(const char* vertexShader, const char* fragmentShader, const char* geometryShader)
  {
    std::string mb = shader::uniformBlock;
    auto vSh = ShaderFromLiteral((mb + vertexShader).c_str(), GL_VERTEX_SHADER);
    auto fSh = ShaderFromLiteral((mb + fragmentShader).c_str(), GL_FRAGMENT_SHADER);
    GLuint gSh = (geometryShader) ? ShaderFromLiteral((mb + geometryShader).c_str(), GL_GEOMETRY_SHADER) : 0;
    const GLuint prog = glCreateProgram();
    glAttachShader(prog, vSh);
    glAttachShader(prog, fSh);
    if (gSh) glAttachShader(prog, gSh);
    glLinkProgram(prog);
    glDetachShader(prog, vSh);
    glDetachShader(prog, fSh);
    glDeleteShader(vSh);
    glDeleteShader(fSh);
    if (gSh) 
    {
      glDetachShader(prog, gSh);
      glDeleteShader(gSh);
    }
    GLchar buf[1024];
    GLsizei length;
    glGetProgramInfoLog(prog, 1024, &length, buf);
    if (length) {
      glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_ERROR, 1, GL_DEBUG_SEVERITY_HIGH, length, buf);
    }
    return prog;
  }


  GLuint LoadTexture(GLenum texUnit, const std::filesystem::path& path) {
    auto FileName = path.string();
    GLint width, height, channels;
    auto texData = stbi_load(FileName.c_str(), &width, &height, &channels, STBI_rgb_alpha);
    if (0 == texData) {
      throw std::runtime_error((std::string("Can't read texture map ") + FileName).c_str());
    }
    GLuint tex = 0; glGenTextures(1, &tex);
    glActiveTexture(texUnit);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texData);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    stbi_image_free(texData);
    return tex;
  }


  // Filenames expected to be +X,-X,+Y,-Y,+Z,-Z
  GLuint LoadCubeMapTexture(GLenum texUnit, const std::vector<std::filesystem::path>& FileNames) {
    GLuint tex = 0; glGenTextures(1, &tex);
    glActiveTexture(texUnit);
    glBindTexture(GL_TEXTURE_CUBE_MAP, tex);
    for (unsigned i = 0; i < 6; ++i) {
      auto FileName = FileNames.at(i).string();
      GLint width, height, channels;
      unsigned char* texData = 0;
      texData = stbi_load(FileName.c_str(), &width, &height, &channels, STBI_rgb);
      if (0 == texData) {
        glDeleteTextures(1, &tex);
        throw std::runtime_error((std::string("Can't read cubemap face '") + FileName).c_str());
      }
      glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, texData);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      stbi_image_free(texData);
    }
    return tex;
  }

}
