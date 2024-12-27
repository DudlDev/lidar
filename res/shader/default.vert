#version 330 core

layout (location = 0) in uvec2 data;

out float angle;
out float distance;

uniform float scale;
uniform int polar;

void main()
{
	// bit manipulation magic (each scan is saved as a u16 then u32 integer, so both have to be extracted from a u64 integer (the data vector))
	uint bit16 = data.x & 0xFFFFu;
	uint bit32 = (data.x & 0xFFFF0000u) >> 0x10u | (data.y & 0xFFFFu) << 0x10u;

	// this conversion is from the official "simple_grabber" application found on github
	angle = float(bit16) * 90.0 / float(1 << 14);
	distance = float(bit32) / 4.0 * 0.001 * scale;

	vec2 p = vec2(sin(angle / 180.0 * 3.1415) * distance / 12.0, cos(angle / 180.0 * 3.1415) * distance / 12.0);

	if(polar == 1)
		gl_Position = vec4(p, 0.0, 1.0);
	else
		gl_Position = vec4(angle / 180.0 - 1.0, distance / 6.0 - 1.0, 0.0, 1.0);
}