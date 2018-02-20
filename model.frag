#version 330
smooth in vec3 vertex_normal;
out vec4 fragColor;
const vec3 lightDir = vec3(0.25, 0.25, -0.25);
const vec3 lightIntensity = vec3(1.0, 1.0, 1.0);
const vec3 ambientIntensity = vec3(0.0, 0.0, 0.0);
const vec3 temp_rgb = vec3(0.0, 0.5, 0.0);
void main()
{
	vec3 surfaceNormal = normalize(vertex_normal);
	float cosAngIncidence = dot(surfaceNormal, lightDir);
	cosAngIncidence = clamp(cosAngIncidence, 0, 1);
	vec3 finalColor = (temp_rgb * lightIntensity * cosAngIncidence)
		+ (temp_rgb * ambientIntensity);
	fragColor = vec4(finalColor, 1.0f);
}
