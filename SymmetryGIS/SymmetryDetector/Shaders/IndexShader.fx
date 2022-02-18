//------------------------------------------------------------------//
// IndexShader -just draw indices of triangles (colors of vertices) //
//------------------------------------------------------------------//

uniform float4x4 matWVP;

//----------------//
// Input / Output //
//----------------//
struct VS_INPUT 
{
   float4 Position : POSITION0;
   float4 Color : COLOR0;
};

struct VS_OUTPUT 
{
   float4 Position : POSITION0;
   float4 Color : COLOR0;
};

texture renderTexture : RenderColorTarget;

sampler targetSampler = sampler_state
{
   Texture = (renderTexture);
};

VS_OUTPUT vs_main( VS_INPUT Input )
{
   VS_OUTPUT Output;
   
   Output.Position = mul( Input.Position, matWVP );
   Output.Color = Input.Color;
      
   return( Output );   
}

float4 ps_main(VS_OUTPUT Input) : COLOR0
{  
   
   return Input.Color;
}




//--------------------------------------------------------------//
// Technique Section for Effect
//--------------------------------------------------------------//
technique Effect
{
   pass BasicModel
   {
      MULTISAMPLEANTIALIAS = false;
      VertexShader = compile vs_2_0 vs_main();
      PixelShader = compile ps_2_0 ps_main();
   }
}