library(ggplot2)
library(optparse)

readAndSplitData <- function(dataPath) {
    tagsPerSheet <- 2
    data <- read.csv(dataPath)
    print(head(data))
    data <- data[order(data$marker_id),]
    tagsId <- unique(data$marker_id)
    nbElements <- ceiling(max(tagsId)/tagsPerSheet)
    result <- list()
    idx <- 1
    for (sheet in seq(1,nbElements)) {
        offset <- (sheet-1) * tagsPerSheet
        startIdx <- offset
        endIdx <- tagsPerSheet+offset-1
        print(which(data$marker_id <= endIdx))
        firstRow <- min(which(data$marker_id >= startIdx))
        lastRow <- max(which(data$marker_id <= endIdx))
        # Add non empty data sets to results
        if (firstRow <= lastRow) {
            print(paste(firstRow,lastRow))
            result[[idx]] <- data[seq(firstRow,lastRow),]
            idx <- idx + 1
        }
    }
    result
}

plotTagsErrors <- function(dataPath, prefix) {
    splittedData <- readAndSplitData(dataPath)
    for (data in splittedData) {
        data$errX = data$obs_x - data$pred_x
        data$errY = data$obs_y - data$pred_y
        g <- ggplot(data, aes(x=errX, y=errY, group=marker_id));
        xAbsMax <- max(-min(data$errX),max(data$errX))
        yAbsMax <- max(-min(data$errY),max(data$errY))
        maxVal <- max(xAbsMax,yAbsMax)
        g <- g + geom_point(size=0.5)
        g <- g + facet_wrap(~marker_id,ncol=2)
        g <- g + coord_cartesian(xlim=c(-xAbsMax,xAbsMax), ylim=c(-yAbsMax,yAbsMax))
        g <- g + theme_bw()
        outputFile = paste0(prefix,"tags_errors_",min(data$marker_id),"-",max(data$marker_id),".png")
        print(outputFile)
        ggsave(outputFile,width=10,height=10)
    }
}

plotVectorsErrors <- function(dataPath, prefix) {
    splittedData <- readAndSplitData(dataPath)
    for (data in splittedData) {
        minTag <- min(data$marker_id)
        maxTag <- max(data$marker_id)
        data$marker_id <- as.factor(data$marker_id)# Setting as factor AFTER computing min and max
        g <- ggplot(data, aes(x=pred_x, y=pred_y, xend=obs_x, yend=obs_y, group="aa", color=marker_id));
        xMin <- 0#min(data$obsX,data$predX)
        xMax <- 644#max(data$obsX,data$predX)
        yMin <- 0#min(data$obsY,data$predY)
        yMax <- 482#max(data$obsY,data$predY)
        # Add label: source: prediction: end: observation
        g <- g + geom_segment(arrow=arrow(length=unit(0.1,"cm")))
        g <- g + coord_cartesian(xlim=c(xMin,xMax), ylim=c(yMin,yMax))
        g <- g + scale_y_reverse()
        g <- g + theme_bw()
        outputFile = paste0(prefix,"vectors_",minTag,"-",maxTag,".png")
        print(outputFile)
        ggsave(outputFile, width=10, height=10)
    }
}


# Script OPTIONS
option_list <- list(
    make_option(c("-p","--prefix"), type="character", default="tmp_",
                help="Output image prefix"),
    make_option(c("-t","--type"), type="character", default="errors",
                help="Type of plot [errors,vectors]")
)
parser <- OptionParser(usage="%prog [options] <logFile>",
                       option_list = option_list)

args <- commandArgs(TRUE)

# Read Options
cmd <- parse_args(parser, args, positional_arguments=1)

input_file <- cmd$args[1]
type <- cmd$option$type
prefix <- cmd$option$prefix

if (type == "errors") {
    plotTagsErrors(input_file, prefix)
} else if (type == "vectors") {
    plotVectorsErrors(input_file, prefix)
} else {
    print(paste0("Invalid option for type: '",type,"'"))
}
