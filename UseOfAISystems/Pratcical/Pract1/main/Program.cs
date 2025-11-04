using Google.GenAI;

string apiKey = "secret_api_key";
var client = new Client(apiKey: apiKey);

var response = await client.Models.GenerateImagesAsync(
  model: "imagen-3.0-generate-002",
  prompt: "NULP is on fire"
);

// Save the image to a file
var image = response.GeneratedImages.First().Image;
await File.WriteAllBytesAsync("skateboard.jpg", image.ImageBytes);