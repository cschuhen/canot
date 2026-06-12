(./build.sh 2>&1; date ) |tee build_output.txt
inotifywait -q -m -e close_write,moved_to,create src | while read -r directory events filename; do
     (./build.sh 2>&1; date ) |tee build_output.txt 
done
