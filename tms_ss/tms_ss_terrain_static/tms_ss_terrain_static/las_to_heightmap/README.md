# las_to_heightmap

## Build

```
docker build -t las2heightmap .
```

## Run
When converting LAS data into terrain data
```
docker run --rm -v .:/data las2heightmap -i /path/to/input.las -o /path/to/output.png -W 2048 -H 2048
```

When LAS data is input, it is converted into a heightmap, and the resulting heightmap is stored in MongoDB.
```
python3 save_image_to_mongodb.py <path to las data>.las  --output <output>.png
```

The "las_to_heightmap" program is based on the "las2heightmap(https://github.com/jhawthorn/las2heightmap)" program.