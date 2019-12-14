uniform sampler2D u_texture;

in vec2 v_texCoord;
out vec4 out_color;

void main()
{
    out_color = texture2D(u_texture, v_texCoord);
}
