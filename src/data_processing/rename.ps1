$sourceFolderPath = Read-Host "Enter the name of the source folder"

# Create the target folder if it doesn't exist
if (-not (Test-Path -Path $sourceFolderPath)) {
    Write-Error $"Source folder {$sourceFolderPath} not found"
    exit -1
}

# Get the list of files in the folder
$files = Get-ChildItem $sourceFolderPath

$index = [int] (Read-Host "Enter the index to start")
if ($index -lt 0) {
    Write-Error "Index must be positive"
    exit -1
}

# Loop through the files and rename them
foreach ($file in $files) {
    # Get the file extension
    $extension = $file.Extension
    $newName = "$index$extension"
    
    try {
        Rename-Item -Path $file.FullName -NewName $newName -Force
    } 
    catch {
        Write-Host "An exception occurred: $($_.Exception.Message)"
    }

    Write-Host "Renamed: $($file.Name) to $newName"

    # Increment the counter
    $index++
}

Write-Host "Renaming complete."
