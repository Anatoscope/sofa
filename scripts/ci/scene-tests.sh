#! /bin/bash

# This scripts tries to run runSofa in batch mode on each .scn file in the
# repository, and saves the results in a bunch of files.
#
# More precisely, it deals with .scn files under the examples/ at the root of
# the source tree, and the .scn files found in the examples/ directory of each
# plugin that was compiled.
#
# The default behaviour it to run 100 iterations for each scene, with a timeout
# of 30 seconds.  This can be influenced via a .scene-tests put directly in one
# of the searched directories, and that contains directives like those:
#
# ignore "path/to/file.scn"
# add "path/to/file.scn"
# timeout "path/to/file.scn" "number-of-seconds"
# iterations "path/to/file.scn" "number-of-iterations"

set -o errexit

usage() {
    echo "Usage: scene-tests.sh [run|count-warnings|count-errors|print-summary] <build-dir> <src-dir>"
}

if [[ "$#" = 3 ]]; then
    command="$1"
    build_dir="$2"
    src_dir="$3"
    output_dir="$build_dir/scene-testing"
else
    usage; exit 1
fi

if [[ ! -d "$build_dir/lib/" ]]; then
    echo "Error: '$build_dir' does not look like a Sofa build."
    usage; exit 1
elif [[ ! -d "$src_dir/applications/plugins" ]]; then
    echo "Error: '$src_dir' does not look like a Sofa source tree."
    usage; exit 1
fi


### Utils

filter-out-comments() {
    sed -e 's/#.*//'
}
remove-leading-blanks() {
    sed -e 's/^[[:blank:]]*//'
}
remove-trailing-blanks() {
    sed -e 's/[[:blank:]]*$//'
}
delete-blank-lines() {
    sed -e '/^$/d'
}
clean-line() {
    filter-out-comments | remove-leading-blanks | remove-trailing-blanks | delete-blank-lines
}
log() {
    # # Send to stderr not to interfere
    tee -a "$output_dir/log.txt" 1>&2
    # cat >> "$output_dir/log.txt"
}

# Well-formed option: 'option "arg 1" "arg 2" "arg 3"'
option-is-well-formed() {
    local cmd='[^[:blank:]]*'
    local arg='"[^"]*"'
    echo "$1" | grep -xqE "^$cmd([[:blank:]]*$arg)+"
}
# $ option-split-args '"a b c" "d" "e f"'
# a b c
# d
# e f
option-split-args() {
    local line="$1"
    local rest="$line"
    while [[ "$rest" != "" ]]; do
	local format='^"\([^"]*\)"[[:blank:]]*\(.*\)'
	local arg=$(echo "$rest" | sed "s/$format/\1/")
	local rest=$(echo "$rest" | sed "s/$format/\2/")
	if [[ "$arg" == "" ]]; then
	    # (This should never happen.)
	    echo "Warning: error parsing arguments: $line" 1>&2
	fi
	echo "$arg"
    done
}
# get-args 'foo "a" "b" "c"'
# "a" "b" "c"
get-args() {
    echo "$1" | sed -e 's/[^[:blank:]][^[:blank:]]*[[:blank:]][[:blank:]]*//'
}
# get-option 'foo "a" "b" "c"'
# foo
get-option() {
    echo "$1" | sed -e 's/\([^[:blank:]][^[:blank:]]*\).*/\1/'
}
# $ get-arg '"a" "b" "c"' 2
# b
get-arg() {
    echo "$1" | option-split-args "$1" | sed -n "$2p"
}
# $ count-args '"a" "b" "c"'
# 3
count-args() {
    option-split-args "$1" | wc -l | tr -d ' '
}

list-scenes() {
    local plugin_src_dir="$1"
    local plugin_bin_dir="$2"

    pushd $plugin_src_dir &> /dev/null
    find . -name "*.scn" -exec sh -c "echo `pwd`/{}:$plugin_bin_dir/{};" \;
    popd &> /dev/null
}

list-plugins() {
    #Grep tous les noms des plugins actifs, puis leurs paths
    grep ^PLUGIN_ ${build_dir}/CMakeCache.txt | grep BOOL=ON$ | cut -d '_' -f2 | cut -d ':' -f1 |
	while  read ligne ; do
        plugin_src_dir=`grep -i ${ligne}_SOURCE_DIR ${build_dir}/CMakeCache.txt | cut -d '=' -f2`
        plugin_bin_dir=`grep -i ${ligne}_BINARY_DIR ${build_dir}/CMakeCache.txt | cut -d '=' -f2`
        echo "$plugin_src_dir":"$plugin_bin_dir"
	done
}

list-scene-directories() {
    # Main directory
    mkdir -p "$output_dir/examples"
    echo "$src_dir/examples:$output_dir/examples" >> "$output_dir/directories.txt"
    # List directories for compiled plugins only
    list-plugins | while read plugin_path; do
         plugin_src_dir=`echo $plugin_path | cut -d ':' -f1`
         plugin_bin_dir=`echo $plugin_path | cut -d ':' -f2`
#          echo "pluginpath",$plugin_path | log
#          echo "plugin_src_dir",$plugin_src_dir | log
#          echo "plugin_bin_dir",$plugin_bin_dir | log
         plugin=`basename $plugin_src_dir`
         echo "$plugin built" | log
         if [ -d "$plugin_src_dir/examples" ]; then
         relative_plugin_binpath=`realpath --relative-to="$build_dir" "$plugin_bin_dir"`
#         relative_plugin_srcpath=`realpath --relative-to="$src_dir" "$plugin_src_dir"`
         mkdir -p "$output_dir/$relative_plugin_binpath/examples"
         echo "$plugin_src_dir/examples:$output_dir/$relative_plugin_binpath/examples"
         else
             echo "Plugin $plugin: no examples/ directory" | log
         fi
   	done >> "$output_dir/directories.txt"
}

create-directories() {
    # List directories where scenes will be tested
    list-scene-directories

    # echo "Creating directory structure."
    # List all scenes
    while read path; do
        plugin_src_dir=`echo $path | cut -d ':' -f1`
        plugin_bin_dir=`echo $path | cut -d ':' -f2`
        rm -f "$plugin_bin_dir/ignore-patterns.txt"
        touch "$plugin_bin_dir/ignore-patterns.txt"
        rm -f "$plugin_bin_dir/add-patterns.txt"
        touch "$plugin_bin_dir/add-patterns.txt"
        list-scenes "$plugin_src_dir" "$plugin_bin_dir" > "$plugin_bin_dir/scenes.txt"

        while read scene; do

            scene_src_path=`echo $scene | cut -d ':' -f1`
#            scene_bin_path=`echo $scene | cut -d ':' -f2`

            relative_scene_path=`realpath --relative-to="$plugin_src_dir" "$scene_src_path"`
            mkdir -p "$plugin_bin_dir/$relative_scene_path"
            if [[ "$CI_BUILD_TYPE" == "Debug" ]]; then
            echo 60 > "$plugin_bin_dir/$relative_scene_path/timeout.txt" # Default debug timeout, in seconds
            else
            echo 30 > "$plugin_bin_dir/$relative_scene_path/timeout.txt" # Default release timeout, in seconds
            fi
            echo 100 > "$plugin_bin_dir/$relative_scene_path/iterations.txt" # Default number of iterations
            echo "$scene":"$plugin_bin_dir/$relative_scene_path" >> "$output_dir/all-scenes.txt"
        done < "$plugin_bin_dir/scenes.txt"
    done < "$output_dir/directories.txt"
}

parse-options-files() {
    echo "Parsing option files."
    while read allpath; do
    plugin_src_dir=`echo $allpath | cut -d ':' -f1`
    plugin_bin_dir=`echo $allpath | cut -d ':' -f2`
    if [[ -e "$plugin_src_dir/.scene-tests" ]]; then
        clean-line < "$plugin_src_dir/.scene-tests" | while read line; do
		if option-is-well-formed "$line"; then
		    local option=$(get-option "$line")
		    local args=$(get-args "$line")
		    case "$option" in
			ignore)
			    if [[ "$(count-args "$args")" = 1 ]]; then
                scene="$(get-arg "$args" 1)"
                echo $scene >> "$plugin_bin_dir/ignore-patterns.txt"
			    else
                echo "$plugin_src_dir/.scene-tests: warning: 'ignore' expects one argument: ignore <pattern>" | log
			    fi
			    ;;
			add)
			    if [[ "$(count-args "$args")" = 1 ]]; then
				scene="$(get-arg "$args" 1)"
                echo $plugin_src_dir/$scene:$plugin_bin_dir/$scene >> "$plugin_bin_dir/add-patterns.txt"
                mkdir -p "$plugin_bin_dir/$scene"
				if [[ "$CI_BUILD_TYPE" == "Debug" ]]; then
                    echo 60 > "$plugin_bin_dir/$scene/timeout.txt" # Default debug timeout, in seconds
				else
                    echo 30 > "$plugin_bin_dir/$scene/timeout.txt" # Default release timeout, in seconds
				fi
                echo 100 > "$plugin_bin_dir/$scene/iterations.txt" # Default number of iterations
			    else
                echo "$plugin_src_dir/.scene-tests: warning: 'add' expects one argument: add <pattern>" | log
			    fi
			    ;;
			timeout)
			    if [[ "$(count-args "$args")" = 2 ]]; then
				scene="$(get-arg "$args" 1)"
                if [[ -e "$plugin_src_dir/$scene" ]]; then
                    get-arg "$args" 2 > "$plugin_bin_dir/$scene/timeout.txt"
				else
                    echo "$plugin_src_dir/.scene-tests: warning: no such file: $scene" | log
				fi
			    else
                echo "$plugin_src_dir/.scene-tests: warning: 'timeout' expects two arguments: timeout <file> <timeout>" | log
			    fi
			    ;;
			iterations)
			    if [[ "$(count-args "$args")" = 2 ]]; then
				scene="$(get-arg "$args" 1)"
                if [[ -e "$plugin_src_dir/$scene" ]]; then
                    get-arg "$args" 2 > "$plugin_bin_dir/$scene/iterations.txt"
				else
                    echo "$plugin_src_dir/.scene-tests: warning: no such file: $scene" | log
				fi
			    else
                echo "$plugin_src_dir/.scene-tests: warning: 'iterations' expects two arguments: iterations <file> <number>" | log
			    fi
			    ;;
			*)
                echo "$plugin_src_dir/.scene-tests: warning: unknown option: $option" | log
			    ;;
		    esac
		else
            echo "$plugin_src_dir/.scene-tests: warning: ill-formed line: $line" | log
		fi
	    done
	fi
    done < "$output_dir/directories.txt"
    
    # echo "Listing ignored and added scenes."
    while read allpath; do
        plugin_bin_dir=`echo $allpath | cut -d ':' -f2`
        grep -f "$plugin_bin_dir/ignore-patterns.txt" \
             "$plugin_bin_dir/scenes.txt" \
             > "$plugin_bin_dir/ignored-scenes.txt" || true
        if [ -s "$plugin_bin_dir/ignore-patterns.txt" ]; then
            grep -vf "$plugin_bin_dir/ignore-patterns.txt" \
             "$plugin_bin_dir/scenes.txt" \
             > "$plugin_bin_dir/tested-scenes.txt" || true
        else
            cp  "$plugin_bin_dir/scenes.txt" "$plugin_bin_dir/tested-scenes.txt"
        fi
        cat "$plugin_bin_dir/ignored-scenes.txt" >> "$output_dir/all-ignored-scenes.txt"

        # Add scenes
        cp "$plugin_bin_dir/add-patterns.txt" "$plugin_bin_dir/added-scenes.txt"
        if [ -s "$plugin_bin_dir/add-patterns.txt" ]; then
            cat "$plugin_bin_dir/add-patterns.txt" \
            >> "$plugin_bin_dir/tested-scenes.txt" || true
            cat "$plugin_bin_dir/add-patterns.txt" \
            >> "$plugin_bin_dir/scenes.txt" || true
        fi
        cat "$plugin_bin_dir/added-scenes.txt" >> "$output_dir/all-added-scenes.txt"
        cat "$plugin_bin_dir/tested-scenes.txt" >> "$output_dir/all-tested-scenes.txt"
    done < "$output_dir/directories.txt"
}

initialize-scene-testing() {
    echo "Initializing scene testing."
    rm -rf "$output_dir"
    mkdir -p "$output_dir"

    runSofa="$(ls "$build_dir/bin/runSofa"{,d,_d} 2> /dev/null || true)"
    if [[ -x "$runSofa" ]]; then
	echo "Found runSofa: $runSofa" | log
    else
	echo "Error: could not find runSofa."
	exit 1
    fi

    touch "$output_dir/warnings.txt"
    touch "$output_dir/errors.txt"

    create-directories
    parse-options-files
}

parallel-execution() {
        #parallel options config
        p_src="$1"
        p_bin="$2"
        src_dir="$3"
        runSofa="$4"

        #Config runSofa and execute
        echo "- $p_src"
        local iterations=$(cat "$p_bin/iterations.txt")
        local options="-g batch -s dag -n $iterations" # -z test
        local runSofa_cmd="$runSofa $options $p_src >> $p_bin/output.txt 2>&1"
        local timeout=$(cat "$p_bin/timeout.txt")
        echo "$runSofa_cmd" > "$p_bin/command.txt"

#        echo $runSofa_cmd | log

        #See Timeout and crashes
        "$src_dir/scripts/ci/timeout.sh" $runSofa "$runSofa_cmd" $timeout "$p_bin/runSofa"
        local status=-1
        if [[ -e $p_bin/runSofa.timeout ]]; then
            echo 'Timeout!'
            echo timeout > "$p_bin/status.txt"
            echo -e "\n\nINFO: Abort caused by timeout.\n" >> "$p_bin/output.txt"
            rm -f $p_bin/runSofa.timeout
        else
            cat $p_bin/runSofa.exit_code > "$p_bin/status.txt"
        fi
        rm -f $p_bin/runSofa.exit_code
}

test-all-scenes() {
    
    p_src=""
    p_bin=""
    echo "Initialize parallel options"
    while read scene; do
        scene_src_path=`echo $scene | cut -d ':' -f1`
        scene_bin_path=`echo $scene | cut -d ':' -f2`
        p_src="$p_src $scene_src_path"
        p_bin="$p_bin $scene_bin_path"
    done < "$output_dir/all-tested-scenes.txt"
    export -f parallel-execution
    echo "Scene testing in progress..."
    parallel --no-notice --xapply parallel-execution ::: $p_src ::: $p_bin ::: $src_dir ::: $runSofa
    echo "Done."
}

extract-warnings() {
    while read scene; do

        scene_src_path=`echo $scene | cut -d ':' -f1`
        scene_bin_path=`echo $scene | cut -d ':' -f2`

        if [[ -e "$scene_bin_path/output.txt" ]]; then
                        sed -ne "/^\[WARNING\] [^]]*/s:\([^]]*\):$scene_src_path\: \1:p \
                    " "$scene_bin_path/output.txt"
        fi
    done < "$output_dir/all-tested-scenes.txt" > "$output_dir/warnings.txt"
}

extract-errors() {
    while read scene; do

        scene_src_path=`echo $scene | cut -d ':' -f1`
        scene_bin_path=`echo $scene | cut -d ':' -f2`

        if [[ -e "$scene_bin_path/output.txt" ]]; then
                        sed -ne "/^\[ERROR\] [^]]*/s:\([^]]*\):$scene_src_path\: \1:p \
                    " "$scene_bin_path/output.txt"
        fi
    done < "$output_dir/all-tested-scenes.txt" > "$output_dir/errors.txt"
}

extract-crashes() {
    while read scene; do

        scene_src_path=`echo $scene | cut -d ':' -f1`
        scene_bin_path=`echo $scene | cut -d ':' -f2`

        if [[ -e "$scene_bin_path/status.txt" ]]; then
            local status="$(cat "$scene_bin_path/status.txt")"
            if [[ "$status" != 0 ]]; then
            echo "$scene_src_path: error: $status"
            fi
        fi
    done < "$output_dir/all-tested-scenes.txt" > "$output_dir/crashes.txt"
}

count-tested-scenes() {
    wc -l < "$output_dir/all-tested-scenes.txt" | tr -d '   '
}

count-warnings() {
    sort "$output_dir/warnings.txt" | uniq | wc -l | tr -d ' '
}

count-errors() {
    wc -l < "$output_dir/errors.txt" | tr -d ' '
}

count-crashes() {
    wc -l < "$output_dir/crashes.txt" | tr -d '   '
}

print-summary() {
    echo "Scene testing summary:"

    cat "$output_dir/time.txt"
    echo # newline

    echo "- $(count-tested-scenes) scene(s) tested"
    echo "- $(count-warnings) warning(s)"

    local errors='$(count-errors)'
    echo "- $(count-errors) error(s)"
    if [[ "$errors" != 0 ]]; then
	while read error; do
	    echo "  - $error"
	done < "$output_dir/errors.txt"
    fi

    local crashes='$(count-crashes)'
    echo "- $(count-crashes) crash(es)"
    if [[ "$crashes" != 0 ]]; then
	while read scene; do

        scene_src_path=`echo $scene | cut -d ':' -f1`
        scene_bin_path=`echo $scene | cut -d ':' -f2`

        if [[ -e "$scene_bin_path/status.txt" ]]; then
        local status="$(cat "$scene_bin_path/status.txt")"
		case "$status" in
		    "timeout")
            echo "  - Timeout: $scene_src_path"
			;;
		    [0-9]*)
			if [[ "$status" -gt 128 && ( $(uname) = Darwin || $(uname) = Linux ) ]]; then
                echo "  - Exit with status $status ($(kill -l $status)): $scene_src_path"
			elif [[ "$status" != 0 ]]; then
                echo "  - Exit with status $status: $scene_src_path"
			fi
			;;
		    *)
            echo "Error: unexpected value in $scene_bin_path/status.txt: $status"
			;;
		esac
	    fi
	done < "$output_dir/all-tested-scenes.txt"
    fi
}

if [[ "$command" = run ]]; then

    time_start=$(date +%s.%N)

    initialize-scene-testing
    test-all-scenes
    extract-warnings
    extract-errors
    extract-crashes

    time_end=$(date +%s.%N)
    time_diff=$(echo "$time_end - $time_start" | bc)
    echo "time:" $time_diff "s - end date: " $(date +%s) "s - " $(date +"%b %d %I:%M %Z") > "$output_dir/time.txt" # duration in s, end date in s and end date in "ctest" format

elif [[ "$command" = print-summary ]]; then
    print-summary
elif [[ "$command" = count-warnings ]]; then
    count-warnings
elif [[ "$command" = count-errors ]]; then
    count-errors
elif [[ "$command" = count-crashes ]]; then
    count-crashes
else
    echo "Unknown command: $command"
fi
